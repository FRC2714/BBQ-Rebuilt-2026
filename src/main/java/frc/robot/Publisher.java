package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

/**
 * Class for publishing useful data to NT that is not necessarily tied to a subsystem. E.g. target
 * pose, robot pose, etc.
 */
public class Publisher {
  DriveSubsystem m_drivetrain;
  Shooter m_shooter;

  public Publisher(DriveSubsystem drivetrain, Shooter shooter) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
  }

  public void publish() {}
}
