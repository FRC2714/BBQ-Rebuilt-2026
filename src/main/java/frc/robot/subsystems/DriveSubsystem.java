// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.Robot;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

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

  double xyStdDev;

  // Publisher for robot pose for use with AdvantageScope
  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisherLLright =
      NetworkTableInstance.getDefault().getStructTopic("poseLLright", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisherLLleft =
      NetworkTableInstance.getDefault().getStructTopic("poseLLleft", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisherLLfront =
      NetworkTableInstance.getDefault().getStructTopic("poseLLfront", Pose2d.struct).publish();

  StructArrayPublisher<Pose3d> tagPosesFrontArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesFront", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> tagPosesLeftArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesLeft", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> tagPosesRightArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesRight", Pose3d.struct)
          .publish();

  StructArrayPublisher<SwerveModuleState> publisherModuleStates =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct)
          .publish();
  StructArrayPublisher<SwerveModuleState> publisherExpectedModuleStates =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Expected Swerve Module States", SwerveModuleState.struct)
          .publish();

  final DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMAXSwerve(
                  DCMotor.getNeoVortex(1), DCMotor.getNeo550(1), COTS.WHEELS.COLSONS.cof, 1))
          .withTrackLengthTrackWidth(
              Inches.of(DriveConstants.kWheelBase), Inches.of(DriveConstants.kTrackWidth))
          .withBumperSize(
              Inches.of(DriveConstants.kWheelBase + 6.0),
              Inches.of(DriveConstants.kTrackWidth + 6.0));

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

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field2d);

    if (Robot.isSimulation()) {
      swerveDriveSimulation =
          new SelfControlledSwerveDriveSimulation(
              new SwerveDriveSimulation(
                  driveTrainSimulationConfig, new Pose2d(3, 3, new Rotation2d())));

      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

      SimulatedArena.getInstance()
          .addDriveTrainSimulation(swerveDriveSimulation.getDriveTrainSimulation());
    }
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
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("OdometryX", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("std dev xy", xyStdDev);
    SmartDashboard.putNumber("omegaRps", omegaRps);

    publisher.set(getPose());
    tagPosesFrontArrayPublisher.set(getCameraTargetPoses3d("limelight-front"));
    tagPosesLeftArrayPublisher.set(getCameraTargetPoses3d("limelight-left"));
    tagPosesRightArrayPublisher.set(getCameraTargetPoses3d("limelight-right"));
    publisherLLfront.set(frontLLMeasurement != null ? frontLLMeasurement.pose : new Pose2d());
    publisherLLleft.set(leftLLMeasurement != null ? leftLLMeasurement.pose : new Pose2d());
    publisherLLright.set(rightLLMeasurement != null ? rightLLMeasurement.pose : new Pose2d());

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

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

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

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
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
  }

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

  public double getAngleToHub() {
    Pose2d pose = getPose().plus(Constants.ShooterConstants.turretOffset);

    Translation2d hub =
        new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32));

    Translation2d robotToHub = hub.minus(pose.getTranslation());
    Rotation2d angleToHub = robotToHub.getAngle();
    Rotation2d turretAngle = angleToHub.minus(pose.getRotation());

    return turretAngle.getDegrees();
  }

  public double getAngleToHubWithLead() {
    Pose2d pose = getPose();
    Translation2d robotVelocity = getFieldRelativeVelocity();

    Translation2d hub =
        new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32));

    Translation2d futurePosition =
        pose.getTranslation().plus(robotVelocity.times(kLatencyCompensation));

    Translation2d toGoal = hub.minus(futurePosition);
    Translation2d targetDirection = toGoal.div(toGoal.getNorm());

    Translation2d targetVelocity = targetDirection.times(kBaselineHorizontalVelocity);
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    Rotation2d turretAngle = shotVelocity.getAngle().minus(pose.getRotation());

    return turretAngle.getDegrees();
  }

  public Pose3d[] getCameraTargetPoses3d(String limelightName) {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
    List<Pose3d> poses = new ArrayList<>();

    for (RawFiducial fiducial : fiducials) {
      AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(fiducial.id).ifPresent(poses::add);
    }

    return poses.toArray(new Pose3d[0]);
  }

  public double getDistanceToHub() {
    Pose2d pose = getPose();
    Translation2d hub =
        new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32));
    return hub.minus(pose.getTranslation()).getNorm();
  }
}
