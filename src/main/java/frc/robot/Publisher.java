package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Class for publishing useful data to NT that is not necessarily tied to a subsystem. E.g. target
 * pose, robot pose, etc.
 */
public class Publisher {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final DyeRotor m_dyeRotor;

  StructPublisher<Pose2d> turretPose =
      NetworkTableInstance.getDefault().getStructTopic("turretPose", Pose2d.struct).publish();

  public Publisher(DriveSubsystem drivetrain, Shooter shooter, Intake intake, DyeRotor dyeRotor) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
    m_dyeRotor = dyeRotor;
  }

  public void publish() {
    turretPose.set(
        new Pose2d(
            m_drivetrain.getPose().getX(),
            m_drivetrain.getPose().getY(),
            m_drivetrain
                .getPose()
                .getRotation()
                .plus(Rotation2d.fromDegrees(m_shooter.getTurretPosition()))));
  }
}
