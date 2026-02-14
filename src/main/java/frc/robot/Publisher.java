package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
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

  public Publisher(DriveSubsystem drivetrain, Shooter shooter, Intake intake) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
  }

  public void publish() {}
}
