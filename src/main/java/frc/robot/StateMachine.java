package frc.robot;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private boolean fuelTrigger = false;

  private State m_state = State.Idle;

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

    fuelBeamBreak =
        m_shooter.flywheelMotorLeader
            .getForwardLimitSwitch(); // Placeholder for actual beam break sensor

    m_publisher = new Publisher(m_drivetrain, m_shooter);
  }

  public void fuelTrue() {
    fuelTrigger = true;
  }

  public void fuelFalse() {
    fuelTrigger = false;
  }

  public Command preload() {
    return m_dyeRotor
        .start()
        .until(() -> fuelTrigger)
        .andThen(m_dyeRotor.stop())
        .withName("preload");
  }

  public Command shoot() {
    // Run preload (dye rotor until fuel loaded, then stop) in parallel with
    // startShooter (spin flywheel until at setpoint). The parallel group finishes
    // when BOTH branches complete. Then start the dye rotor again to feed the shot.
    return preload()
        .alongWith(m_shooter.startShooter().until(m_shooter::flywheelAtSetpoint))
        .andThen(m_dyeRotor.start())
        .withName("shoot");

    // return new InstantCommand(
    //     () -> {
    //       preload()
    //           // .alongWith(m_shooter.startShooter())
    //           // .until(m_shooter::flywheelAtSetpoint)
    //           .withTimeout(2)
    //           .andThen(m_dyeRotor.start())
    //           .withName("shoot");
    //     });
  }

  public State getState() {
    return m_state;
  }

  public void setState(State state) {
    m_state = state;
  }

  @Override
  public void periodic() {
    fuelTrigger = fuelBeamBreak.isPressed();
    if (fuelTrigger) {
      fuelTrue();
    } else {
      fuelFalse();
    }
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

    SmartDashboard.putBoolean("Fuel Loaded", fuelTrigger);
    m_publisher.publish();
  }
}
