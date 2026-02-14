package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Publisher m_publisher;

  private State m_state = State.Idle;

  enum State {
    Idle,
    Shooting,
    Climbing
  }

  public StateMachine(DriveSubsystem drivetrain, Shooter shooter) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;

    m_publisher = new Publisher(m_drivetrain, m_shooter);
  }

  public State getState() {
    return m_state;
  }

  public void setState(State state) {
    m_state = state;
  }

  @Override
  public void periodic() {
    Translation2d virtualTarget = m_drivetrain.getVirtualTarget();
    Translation2d robotPosition = m_drivetrain.getPose().getTranslation();

    double distanceToTarget = virtualTarget.getDistance(robotPosition);

    // Calculate robot-relative angle to virtual target
    Translation2d robotToTarget = virtualTarget.minus(robotPosition);
    Rotation2d fieldAngle = robotToTarget.getAngle();
    Rotation2d turretAngle = fieldAngle.minus(m_drivetrain.getPose().getRotation());

    m_shooter.updateTurretTarget(turretAngle.getDegrees());
    m_shooter.updateHoodTarget(distanceToTarget);
    m_shooter.updateFlywheelTarget(distanceToTarget);

    m_publisher.publish();
  }
}
