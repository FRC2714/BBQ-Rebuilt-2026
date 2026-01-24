// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

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

  private Pose2d m_previousPose = new Pose2d();
  private double m_previousTimestamp = 0;
  private double m_robotVelocityX = 0;
  private double m_robotVelocityY = 0;

  private static final double kShotSpeedMetersPerSecond = 12.0;
  private static final double kDragCorrectionFactor = 1.15;
  private static final double kMinLeadDistance = 0.5;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field2d);
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

    LimelightHelpers.SetRobotOrientation("limelight-back", getHeading(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-front", getHeading(), 0, 0, 0, 0, 0);

    double omegaRps = Units.degreesToRotations(getTurnRate());

    var frontLLMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
    var backLLMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back");

    if (Math.abs(omegaRps) < 2.0) {
      if (frontLLMeasurement != null && frontLLMeasurement.tagCount > 0) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(
            frontLLMeasurement.pose, frontLLMeasurement.timestampSeconds);
      }

      if (backLLMeasurement != null && backLLMeasurement.tagCount > 0) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        m_poseEstimator.addVisionMeasurement(
            backLLMeasurement.pose, backLLMeasurement.timestampSeconds);
      }
    }

    double currentTime = Timer.getFPGATimestamp();
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    double dt = currentTime - m_previousTimestamp;

    if (dt > 0.005 && dt < 0.1) {
      m_robotVelocityX = (currentPose.getX() - m_previousPose.getX()) / dt;
      m_robotVelocityY = (currentPose.getY() - m_previousPose.getY()) / dt;
    }

    m_previousPose = currentPose;
    m_previousTimestamp = currentTime;

    m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("OdometryX", m_poseEstimator.getEstimatedPosition().getX());
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

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw() >= 0 ? m_gyro.getYaw() * 360 : Math.abs((m_gyro.getYaw() * 360) + 360);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityYaw() * 360 * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Translation2d getRobotVelocity() {
    return new Translation2d(m_robotVelocityX, m_robotVelocityY);
  }

  public double getRobotSpeed() {
    return Math.hypot(m_robotVelocityX, m_robotVelocityY);
  }

  public double getAngleToHub() {
    Pose2d pose = getPose();

    // Hub position in field coordinates (meters)
    Translation2d hub =
        new Translation2d(
            edu.wpi.first.math.util.Units.inchesToMeters(468.56),
            edu.wpi.first.math.util.Units.inchesToMeters(158.32));

    // Vector from robot to hub
    Translation2d robotToHub = hub.minus(pose.getTranslation());

    // Field-relative angle to hub
    Rotation2d angleToHub = robotToHub.getAngle();

    // Turret angle relative to robot forward
    Rotation2d turretAngle = angleToHub.minus(pose.getRotation());

    // Return degrees (wrapped to [-180, 180])
    return turretAngle.getDegrees();
  }

  public double getAngleToHubWithLead() {
    Pose2d pose = getPose();

    Translation2d hub =
        new Translation2d(
            Units.inchesToMeters(468.56),
            Units.inchesToMeters(158.32));

    Translation2d robotToHub = hub.minus(pose.getTranslation());
    double distanceToHub = robotToHub.getNorm();

    if (distanceToHub < kMinLeadDistance) {
      return getAngleToHub();
    }

    double flightTime = (distanceToHub / kShotSpeedMetersPerSecond) * kDragCorrectionFactor;

    Translation2d leadOffset =
        new Translation2d(
            m_robotVelocityX * flightTime,
            m_robotVelocityY * flightTime);

    Translation2d aimPoint = hub.minus(leadOffset);
    Translation2d robotToAimPoint = aimPoint.minus(pose.getTranslation());
    Rotation2d angleToAimPoint = robotToAimPoint.getAngle();
    Rotation2d turretAngle = angleToAimPoint.minus(pose.getRotation());

    return turretAngle.getDegrees();
  }

  public double getDistanceToHub() {
    Pose2d pose = getPose();
    Translation2d hub =
        new Translation2d(
            Units.inchesToMeters(468.56),
            Units.inchesToMeters(158.32));
    return hub.minus(pose.getTranslation()).getNorm();
  }
}