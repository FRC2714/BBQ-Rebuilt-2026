package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  DriveSubsystem m_drivetrain;
  Shooter m_shooter;

  public StateMachine(DriveSubsystem drivetrain, Shooter shooter) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
  }

  @Override
  public void periodic() {
    m_shooter.updateTurretTarget(m_drivetrain.getAngleToHub());
  }
}
