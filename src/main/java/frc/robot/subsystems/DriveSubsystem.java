// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Field;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.Simulation;
import frc.robot.utils.LimelightHelpers;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;

public class DriveSubsystem extends SubsystemBase {
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  private final Canandgyro m_gyro = new Canandgyro(0);

  private final Field2d m_field2d = new Field2d();

  private double m_driverHeadingOffsetDeg = 0.0; // Used for relative heading for the driver

  private static final double[] BLUE_ZONE = {
    0.0, 0.0, FieldConstants.LinesVertical.allianceZone, FieldConstants.fieldWidth
  };
  private static final double[] RED_ZONE = {
    FieldConstants.LinesVertical.oppAllianceZone,
    0.0,
    FieldConstants.fieldLength,
    FieldConstants.fieldWidth
  };

  private static final double ROBOT_BUFFER =
      Units.inchesToMeters((DriveConstants.kTrackWidth + 6.0) / 2.0);

  private boolean isInZone(double[] zone) {
    Translation2d pos = getPose().getTranslation();
    return pos.getX() + ROBOT_BUFFER >= zone[0]
        && pos.getY() + ROBOT_BUFFER >= zone[1]
        && pos.getX() - ROBOT_BUFFER <= zone[2]
        && pos.getY() - ROBOT_BUFFER <= zone[3];
  }

  public boolean isInAllianceZone() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    return isInZone(isRed ? RED_ZONE : BLUE_ZONE);
  }

  double xyStdDev;

  private boolean shooting = false;
  

  // Publisher for robot pose for use with AdvantageScope
  StructArrayPublisher<SwerveModuleState> publisherModuleStates =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct)
          .publish();
  StructArrayPublisher<SwerveModuleState> publisherExpectedModuleStates =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Expected Swerve Module States", SwerveModuleState.struct)
          .publish();

  SelfControlledSwerveDriveSimulation swerveDriveSimulation;

  // Odometry class for tracking robot pose
  public SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d(3, 3, new Rotation2d()),
          LimelightConstants.m_stateStdDevs,
          LimelightConstants.m_visionStdDevs);

  private static final double kLatencyCompensation = 0.1;
  private static final double kBaselineHorizontalVelocity = 6.0;

  private double m_driveSysIdVoltage = 0.0;
  private double m_rotationSysIdVoltage = 0.0;

  private final SysIdRoutine rotationRoutine;
  private final SysIdRoutine driveRoutine;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field2d);

    if (Robot.isSimulation()) {
      swerveDriveSimulation =
          new SelfControlledSwerveDriveSimulation(
              Simulation.getInstance().getSwerveDriveSimulation());
    }

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    driveRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(2.5)),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)),
                null, // URCL handles logging
                this,
                "drive"));

    rotationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.driveVoltageRotateTest(voltage.in(Volts)),
                null, // URCL handles logging
                this,
                "rotation"));
  }

  public SysIdRoutine sysIdDrive() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageForwardTest(voltage.in(Volts)),
            null, // URCL handles logging
            this,
            "drive"));
  }

  public SysIdRoutine sysIdRotation() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.driveVoltageRotateTest(voltage.in(Volts)),
            null, // URCL handles logging
            this,
            "rotation"));
  }

  public Command translationalQuasistatic() {
    return new SequentialCommandGroup(
        driveRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command rotationalQuasistatic() {
    return new SequentialCommandGroup(
        rotationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        rotationRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command translationalDynamic() {
    return new SequentialCommandGroup(
        driveRoutine.dynamic(SysIdRoutine.Direction.kForward),
        driveRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command rotationalDynamic() {
    return new SequentialCommandGroup(
        rotationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        rotationRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  private void driveVoltageForwardTest(double voltage) {
    m_driveSysIdVoltage = voltage;
    m_frontLeft.setVoltageAngle(voltage, new Rotation2d());
    m_frontRight.setVoltageAngle(voltage, new Rotation2d());
    m_rearLeft.setVoltageAngle(voltage, new Rotation2d());
    m_rearRight.setVoltageAngle(voltage, new Rotation2d());
  }

  private void driveVoltageRotateTest(double voltage) {
    m_rotationSysIdVoltage = voltage;
    m_frontLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(-27.9));
    m_frontRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(27.9));
    m_rearLeft.setVoltageAngle(-voltage, Rotation2d.fromDegrees(27.9));
    m_rearRight.setVoltageAngle(voltage, Rotation2d.fromDegrees(-27.9));
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    LimelightHelpers.SetRobotOrientation("limelight-right", getHeading(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-front", getHeading(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left", getHeading(), 0, 0, 0, 0, 0);

    LimelightHelpers.Flush();

    double omegaRps = Units.degreesToRotations(getTurnRate());

    var frontLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    var leftLLMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    var rightLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

    if (Math.abs(omegaRps) < .7) {
      if (frontLLMeasurement != null && frontLLMeasurement.tagCount > 0) {
        xyStdDev = .7 * (1 + frontLLMeasurement.avgTagDist * .5);
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        m_poseEstimator.addVisionMeasurement(
            frontLLMeasurement.pose, frontLLMeasurement.timestampSeconds);
      }

      if (leftLLMeasurement != null && leftLLMeasurement.tagCount > 0) {
        xyStdDev = .7 * (1 + leftLLMeasurement.avgTagDist * .5);
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        m_poseEstimator.addVisionMeasurement(
            leftLLMeasurement.pose, leftLLMeasurement.timestampSeconds);
      }
      if (rightLLMeasurement != null && rightLLMeasurement.tagCount > 0) {
        xyStdDev = .7 * (1 + rightLLMeasurement.avgTagDist * .5);
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        m_poseEstimator.addVisionMeasurement(
            rightLLMeasurement.pose, rightLLMeasurement.timestampSeconds);
      }
    }

    m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());

    if (Robot.isReal()) {
      publisherModuleStates.set(
          new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
          });
    } else {
      publisherModuleStates.set(swerveDriveSimulation.getMeasuredStates());
    }
  }

  @Override
  public void simulationPeriodic() {
    if (swerveDriveSimulation != null) {
      swerveDriveSimulation.periodic();
    }
  }

  public Pose2d getPose() {
    if (swerveDriveSimulation != null) {
      return swerveDriveSimulation.getActualPoseInSimulationWorld();
    }

    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    if (swerveDriveSimulation != null) {
      swerveDriveSimulation.resetOdometry(pose);
    }

    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    double driverRelativeHeading = getHeading() - m_driverHeadingOffsetDeg;

    if (shooting) {
      xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond / 2;
      ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond / 2;
      rotDelivered = rot * DriveConstants.kMaxAngularSpeed / 2;
    }

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(driverRelativeHeading))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    publisherExpectedModuleStates.set(swerveModuleStates);

    if (this.swerveDriveSimulation != null) {
      this.swerveDriveSimulation.runSwerveStates(swerveModuleStates);
      // this.swerveDriveSimulation.runChassisSpeeds(
      // new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
      // new Translation2d(),
      // fieldRelative,
      // true);
      return;
    }

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = speeds.vxMetersPerSecond;
    double ySpeedDelivered = speeds.vyMetersPerSecond;
    double rotDelivered = speeds.omegaRadiansPerSecond;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(getHeading()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    publisherExpectedModuleStates.set(swerveModuleStates);

    if (this.swerveDriveSimulation != null) {
      this.swerveDriveSimulation.runSwerveStates(swerveModuleStates);
      // this.swerveDriveSimulation.runChassisSpeeds(
      //     new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
      //     new Translation2d(),
      //     fieldRelative,
      //     true);
      return;
    }

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setShootingStateTrue() {
    shooting = true;
  }

  public void setShootingStateFalse() {
    shooting = false;
  }


  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    if (swerveDriveSimulation != null) {
      swerveDriveSimulation.runSwerveStates(desiredStates);
      return;
    }

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroPose() {
    Pose2d pose = new Pose2d();

    m_gyro.setYaw(pose.getRotation().getRotations());
    m_poseEstimator.resetRotation(pose.getRotation());
    m_poseEstimator.resetPosition(
        pose.getRotation(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);

    LimelightHelpers.SetRobotOrientation("limelight-front", 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left", 0, 0, 0, 0, 0, 0);

    LimelightHelpers.SetIMUMode("limelight-front", 1);
    LimelightHelpers.SetIMUMode("limelight-right", 1);
    LimelightHelpers.SetIMUMode("limelight-left", 1);

    // Switch back to fused mode after seeding
    new WaitCommand(0.1)
        .andThen(new InstantCommand(() -> LimelightHelpers.SetIMUMode("limelight-front", 4)))
        .alongWith(new InstantCommand(() -> LimelightHelpers.SetIMUMode("limelight-right", 4)))
        .alongWith(new InstantCommand(() -> LimelightHelpers.SetIMUMode("limelight-left", 4)));
  }

  public void zeroDriverHeading() {
    m_driverHeadingOffsetDeg = getHeading();
  }

  public void setDriverHeadingOffset(double offsetDeg) {
    m_driverHeadingOffsetDeg = getHeading() - offsetDeg;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    if (swerveDriveSimulation != null) {
      return swerveDriveSimulation.getActualPoseInSimulationWorld().getRotation().getDegrees();
    }

    return m_poseEstimator == null
        ? Units.rotationsToDegrees(m_gyro.getYaw())
        : m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    if (swerveDriveSimulation != null) {
      return swerveDriveSimulation
          .getDriveTrainSimulation()
          .getGyroSimulation()
          .getMeasuredAngularVelocity()
          .in(DegreesPerSecond);
    }

    return m_gyro.getAngularVelocityYaw() * 360 * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    if (swerveDriveSimulation != null) {
      return swerveDriveSimulation.getActualSpeedsRobotRelative();
    }

    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public Translation2d getFieldRelativeVelocity() {
    ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
    Rotation2d heading = Rotation2d.fromDegrees(getHeading());
    return new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
        .rotateBy(heading);
  }

  public Translation2d getVirtualTarget() {
    Pose2d pose = getPose();
    Translation2d robotVelocity = getFieldRelativeVelocity();
    Translation2d hub = Field.getAllianceHub().toTranslation2d();

    Translation2d futurePosition =
        pose.getTranslation().plus(robotVelocity.times(kLatencyCompensation));

    Translation2d toGoal = hub.minus(futurePosition);
    Translation2d targetDirection = toGoal.div(toGoal.getNorm());

    Translation2d targetVelocity = targetDirection.times(kBaselineHorizontalVelocity);
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    double distanceToHub = toGoal.getNorm();
    Translation2d virtualTarget =
        futurePosition.plus(shotVelocity.div(shotVelocity.getNorm()).times(distanceToHub));

    return virtualTarget;
  }
}
