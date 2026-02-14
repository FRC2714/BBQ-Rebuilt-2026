package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Publisher m_publisher;
  private final DyeRotor m_dyeRotor;

  public StateMachine(DriveSubsystem drivetrain, Shooter shooter, DyeRotor dyeRotor) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_dyeRotor = dyeRotor;

    m_publisher = new Publisher(m_drivetrain, m_shooter);
  }

  StructArrayPublisher<Pose3d> publisherZeroedComponentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> publisherFinalComponentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("FinalComponentPoses", Pose3d.struct)
          .publish();

  @Override
  public void periodic() {
    m_shooter.updateTurretTarget(m_drivetrain.getAngleToHub());

    m_publisher.publish();

    Pose3d[] zeroRobotPose = new Pose3d[1];
    for (int i = 0; i < zeroRobotPose.length; i++) {
      zeroRobotPose[i] = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    }
    publisherZeroedComponentPoses.set(zeroRobotPose);
    Pose3d[] finalRobotPose = new Pose3d[] {m_dyeRotor.getDyePose()};
    publisherFinalComponentPoses.set(finalRobotPose);
  }
}
;
