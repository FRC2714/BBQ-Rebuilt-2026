package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Generalization of updating the targets
  private void aimAt(Translation2d target, Translation2d robotPosition, Rotation2d robotHeading) {
    Translation2d robotToTarget = target.minus(robotPosition);
    Rotation2d fieldAngle = robotToTarget.getAngle();
    Rotation2d turretAngle = fieldAngle.minus(robotHeading);

    m_shooter.updateTurretTarget(turretAngle.getDegrees());
    m_shooter.updateHoodTarget(robotToTarget.getNorm());
    m_shooter.updateFlywheelTarget(robotToTarget.getNorm());
  
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
    // Zone based targeting with travel time calculations
    Translation2d robotPosition = m_drivetrain.getPose().getTranslation();
    Rotation2d robotHeading = m_drivetrain.getPose().getRotation();

    if (m_drivetrain.isInAllianceZone()) {
      Translation2d virtualTarget = m_drivetrain.getVirtualTarget();
      aimAt(virtualTarget, robotPosition, robotHeading);
      m_publisher.publish();
      return;
    }

    double airstrikeX = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/x", 0));
    double airstrikeY = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/y", 0));

    if (airstrikeX == 0 && airstrikeY == 0) {
      m_shooter.updateTurretTarget(0.0);
      m_publisher.publish();
      return;
    }

    Translation2d airstrikeTarget = new Translation2d(airstrikeX, airstrikeY);
    aimAt(airstrikeTarget, robotPosition, robotHeading);
    m_publisher.publish();
  }
}
