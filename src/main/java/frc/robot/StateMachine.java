package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Publisher m_publisher;
  private final Intake m_intake;

  private static State m_state = State.Idle;

  private static boolean isNotClimbing() {
    return !(m_state == State.Climbing);
  }

  enum State {
    Idle,
    Shooting,
    Climbing
  }

  public StateMachine(DriveSubsystem drivetrain, Shooter shooter, Intake intake) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;

    m_publisher = new Publisher(m_drivetrain, m_shooter, m_intake);
  }

  public State getState() {
    return m_state;
  }

  public void setState(State state) {
    m_state = state;
  }

  // intake commands

  public Command intakeSequence() {
    return (m_intake.intake().onlyIf(StateMachine::isNotClimbing));
  }

  public Command extakeSequence() {
    return (m_intake.extake().onlyIf(StateMachine::isNotClimbing));
  }

  public Command stowSequence() {
    return (m_intake.stow().onlyIf(StateMachine::isNotClimbing));
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
