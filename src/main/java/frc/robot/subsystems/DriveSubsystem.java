// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
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

  // The gyro sensor
  private final Canandgyro m_gyro = new Canandgyro(0);

  private final Field2d m_field2d = new Field2d();

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
          new Pose2d(),
          LimelightConstants.m_stateStdDevs,
          LimelightConstants.m_visionStdDevs);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    SmartDashboard.putData("Field", m_field2d);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    LimelightHelpers.SetRobotOrientation("limelight-back", getHeading(), 0, 0, 0, 0, 0);

    LimelightHelpers.SetRobotOrientation("limelight-front", getHeading(), 0, 0, 0, 0, 0);

    double omegaRps = Units.degreesToRotations(getTurnRate());
    var frontLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    var backLLMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

    if (backLLMeasurement != null && backLLMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
      m_poseEstimator.addVisionMeasurement(
          backLLMeasurement.pose, backLLMeasurement.timestampSeconds);
    } else if (frontLLMeasurement != null
        && frontLLMeasurement.tagCount > 0
        && Math.abs(omegaRps) < 2.0) {
      m_poseEstimator.addVisionMeasurement(
          frontLLMeasurement.pose, frontLLMeasurement.timestampSeconds);
      // m_robotContainer.m_robotDrive.resetOdometry(frontLLMeasurement.pose);
    }

    if (backLLMeasurement != null
        && frontLLMeasurement != null
        && frontLLMeasurement.tagCount > 0
        && backLLMeasurement.tagCount > 0
        && Math.abs(omegaRps) < 2.0) {
      Pose2d avgPose =
          new Pose2d(
              (frontLLMeasurement.pose.getX() + backLLMeasurement.pose.getX()) / 2,
              (frontLLMeasurement.pose.getY() + backLLMeasurement.pose.getY()) / 2,
              (Rotation2d.fromDegrees(
                  ((frontLLMeasurement.pose.getRotation()).getDegrees()
                          + backLLMeasurement.pose.getRotation().getDegrees())
                      / 2)));
      //  m_poseEstimator.addVisionMeasurement(avgPose, frontLLMeasurement.timestampSeconds);
      // m_robotContainer.m_robotDrive.resetOdometry(avgPose);
    }
    m_poseEstimator.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("OdometryX", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("OdometryY", m_poseEstimator.getEstimatedPosition().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
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

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
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

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw() >= 0 ? m_gyro.getYaw() * 360 : Math.abs((m_gyro.getYaw() * 360) + 360);
  }

  public Command alignDrive(XboxController controller, Supplier<Pose2d> targetPoseSupplier) {
    PIDController turnController = new PIDController(.1, 0.0, 0.0);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          double controllerVelX = -controller.getLeftY();
          double controllerVelY = -controller.getLeftX();
          Pose2d drivePose = getPose(); // MaxSwerve: getPose() instead of getState().Pose
          Pose2d targetPose = targetPoseSupplier.get();

          Rotation2d desiredAngle =
              drivePose.relativeTo(targetPose).getTranslation().getAngle().plus(Rotation2d.k180deg);

          Rotation2d currentAngle = drivePose.getRotation();
          Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
          double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

          if ((Math.abs(wrappedAngleDeg) < 1) && Math.hypot(controllerVelX, controllerVelY) < .1) {
            // MaxSwerve: Lock wheels in X formation
            setX(); // Stop
            // Or: setX(); if you have that method
          } else {
            double rotationalRate =
                turnController.calculate(currentAngle.getRadians(), desiredAngle.getRadians());

            // MaxSwerve: Call drive() method directly
            drive(
                controllerVelX * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                controllerVelY * Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                rotationalRate * Constants.DriveConstants.kMaxAngularSpeed,
                true // field-relative
                );
          }
        },
        this); // 'this' = drivetrain subsystem for requirements
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityYaw() * 360 * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
