package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Publisher m_publisher;

  public StateMachine(DriveSubsystem drivetrain, Shooter shooter) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;

    m_publisher = new Publisher(m_drivetrain, m_shooter);
  }

  @Override
  public void periodic() {
    m_shooter.updateTurretTarget(m_drivetrain.getAngleToHub());

    m_publisher.publish();
  }
}
