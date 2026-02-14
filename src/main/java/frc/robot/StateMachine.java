package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private void aimAt(Translation2d target, Translation2d robotPosition, Rotation2d robotHeading) {
    Translation2d robotToTarget = target.minus(robotPosition);
    Rotation2d fieldAngle = robotToTarget.getAngle();
    Rotation2d turretAngle = fieldAngle.minus(robotHeading);

    m_shooter.updateTurretTarget(turretAngle.getDegrees());
    m_shooter.updateHoodTarget(robotToTarget.getNorm());
    m_shooter.updateFlywheelTarget(robotToTarget.getNorm());
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
