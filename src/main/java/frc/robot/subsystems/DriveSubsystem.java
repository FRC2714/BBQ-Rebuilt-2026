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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;

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

  private static final double kLatencyCompensation = 0.1;
  private static final double kBaselineHorizontalVelocity = 6.0;

  public DriveSubsystem() {
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

    m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("OdometryX", m_poseEstimator.getEstimatedPosition().getX());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

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
    return m_gyro.getYaw() >= 0 ? m_gyro.getYaw() * 360 : Math.abs((m_gyro.getYaw() * 360) + 360);
  }

  public double getTurnRate() {
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
    Pose2d pose = getPose();

    Translation2d hub =
        new Translation2d(
            Units.inchesToMeters(468.56),
            Units.inchesToMeters(158.32));

    Translation2d robotToHub = hub.minus(pose.getTranslation());
    Rotation2d angleToHub = robotToHub.getAngle();
    Rotation2d turretAngle = angleToHub.minus(pose.getRotation());

    return turretAngle.getDegrees();
  }

  public double getAngleToHubWithLead() {
    Pose2d pose = getPose();
    Translation2d robotVelocity = getFieldRelativeVelocity();

    Translation2d hub =
        new Translation2d(
            Units.inchesToMeters(468.56),
            Units.inchesToMeters(158.32));

    Translation2d futurePosition =
        pose.getTranslation().plus(robotVelocity.times(kLatencyCompensation));

    Translation2d toGoal = hub.minus(futurePosition);
    Translation2d targetDirection = toGoal.div(toGoal.getNorm());

    Translation2d targetVelocity = targetDirection.times(kBaselineHorizontalVelocity);
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    Rotation2d turretAngle = shotVelocity.getAngle().minus(pose.getRotation());

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