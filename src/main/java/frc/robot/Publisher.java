package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Vision;

/**
 * Class for publishing useful data to NT that is not necessarily tied to a subsystem. E.g. target
 * pose, robot pose, etc.
 */
public class Publisher {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final DyeRotor m_dyeRotor;

  /** 3d sim */
  StructArrayPublisher<Pose3d> publisherZeroedComponentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> publisherFinalComponentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("FinalComponentPoses", Pose3d.struct)
          .publish();

  /** Turret things */
  StructPublisher<Pose2d> virtualTarget =
      NetworkTableInstance.getDefault().getStructTopic("virtualTarget", Pose2d.struct).publish();

  StructPublisher<Pose2d> turretPose =
      NetworkTableInstance.getDefault().getStructTopic("turretPose", Pose2d.struct).publish();

  /** Pose */
  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

  DoubleArrayPublisher publisherPoseArray =
      NetworkTableInstance.getDefault().getDoubleArrayTopic("Robot Pose Array").publish();

  StructPublisher<Pose2d> publisherLLfrontRight =
      NetworkTableInstance.getDefault().getStructTopic("poseLLfrontRight", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisherLLfrontLeft =
      NetworkTableInstance.getDefault().getStructTopic("poseLLfrontLeft", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisherLLrearLeft =
      NetworkTableInstance.getDefault().getStructTopic("poseLLrearLeft", Pose2d.struct).publish();

  StructPublisher<Pose2d> publisherLLrearRight =
      NetworkTableInstance.getDefault().getStructTopic("poseLLrearRight", Pose2d.struct).publish();

  StructArrayPublisher<Pose3d> tagPosesFrontRightArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesFrontRight", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> tagPosesFrontLeftArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesFrontLeft", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> tagPosesRearLeftArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesRearLeft", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> tagPosesRearRightArrayPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("tagPosesRearRight", Pose3d.struct)
          .publish();

  public Publisher(DriveSubsystem drivetrain, Shooter shooter, Intake intake, DyeRotor dyeRotor) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
    m_dyeRotor = dyeRotor;
  }

  /** Publishes all NetworkTables data: poses, turret, Limelight measurements, and sim fuel. */
  public void publish() {
    Pose3d[] zeroRobotPose = new Pose3d[4];
    for (int i = 0; i < zeroRobotPose.length; i++) {
      zeroRobotPose[i] = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    }
    publisherZeroedComponentPoses.set(zeroRobotPose);
    Pose3d[] finalRobotPose =
        new Pose3d[] {
          m_dyeRotor.getPose3d(),
          m_shooter.getTurretPose3d(),
          m_shooter.getHoodPose3d(),
          m_shooter.getFlyWheelPose3d(),
          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0))
        };
    publisherFinalComponentPoses.set(finalRobotPose);

    turretPose.set(
        new Pose2d(
            m_drivetrain.getPose().getX(),
            m_drivetrain.getPose().getY(),
            m_drivetrain
                .getPose()
                .getRotation()
                .plus(Rotation2d.fromDegrees(m_shooter.getTurretPosition()))));

    virtualTarget.set(new Pose2d(m_drivetrain.getVirtualTarget(), new Rotation2d()));

    publisher.set(m_drivetrain.getPose());
    Pose2d pose = m_drivetrain.getPose();
    publisherPoseArray.set(
        new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    var frontRightLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kFrontRightName);
    var frontLeftLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kFrontLeftName);
    var rearLeftLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kRearLeftName);
    var rearRightLLMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kRearRightName);
    publisherLLfrontRight.set(
        frontRightLLMeasurement != null ? frontRightLLMeasurement.pose : new Pose2d());
    publisherLLfrontLeft.set(
        frontLeftLLMeasurement != null ? frontLeftLLMeasurement.pose : new Pose2d());
    publisherLLrearLeft.set(
        rearLeftLLMeasurement != null ? rearLeftLLMeasurement.pose : new Pose2d());
    publisherLLrearRight.set(
        rearRightLLMeasurement != null ? rearRightLLMeasurement.pose : new Pose2d());
    tagPosesFrontRightArrayPublisher.set(
        Vision.getCameraTargetPoses3d(LimelightConstants.kFrontRightName));
    tagPosesFrontLeftArrayPublisher.set(
        Vision.getCameraTargetPoses3d(LimelightConstants.kFrontLeftName));
    tagPosesRearLeftArrayPublisher.set(
        Vision.getCameraTargetPoses3d(LimelightConstants.kRearLeftName));
    tagPosesRearRightArrayPublisher.set(
        Vision.getCameraTargetPoses3d(LimelightConstants.kRearRightName));

    if (Robot.isSimulation()) Simulation.getInstance().publish();
  }
}
