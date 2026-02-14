package frc.robot;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final DyeRotor m_dyeRotor;
  private final Publisher m_publisher;

  SparkLimitSwitch fuelBeamBreak;

  private static State m_state = State.Idle;

  private static boolean isNotClimbing() {
    return !(m_state == State.Climbing);
  }

  enum State {
    Idle,
    Shooting,
    Climbing
  }

  public StateMachine(
      DriveSubsystem drivetrain, Shooter shooter, Intake intake, DyeRotor dyeRotor) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
    m_dyeRotor = dyeRotor;

    m_publisher = new Publisher(m_drivetrain, m_shooter, m_intake);
  }

  public Command preload() {
    return m_dyeRotor
        .start()
        .until(() -> m_shooter.getFuelLimitSwitch())
        .andThen(m_dyeRotor.stop())
        .withName("preload")
        .andThen(() -> m_state = State.Idle);
  }

  public Command shoot() {
    // Run preload (dye rotor until fuel loaded, then stop) in parallel with
    // startShooter (spin flywheel until at setpoint). The parallel group finishes
    // when BOTH branches complete. Then start the dye rotor again to feed the shot.
    return preload()
        .alongWith(m_shooter.startShooter().until(m_shooter::flywheelAtSetpoint))
        .andThen(m_dyeRotor.start())
        .withName("shoot")
        .andThen(() -> m_state = State.Shooting);
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
    // fuelTrigger = fuelBeamBreak.isPressed();

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

    SmartDashboard.putString(
        "State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

    SmartDashboard.putString("State", m_state.toString());

    m_publisher.publish();
  }
}
