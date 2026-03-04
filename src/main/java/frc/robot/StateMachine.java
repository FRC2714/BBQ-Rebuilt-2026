package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Coordinates all robot subsystems through a state machine (Idle, Shooting, Climbing). Commands are
 * guarded by state checks to prevent conflicting actions.
 */
public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final DyeRotor m_dyeRotor;
  private final Publisher m_publisher;
  private final XboxController m_driverHID;
  private final Climb m_climb;

  private static State m_state = State.Idle;
  private boolean phaseShiftActive = false;
  private boolean phaseShiftWarningActive = false;
  private boolean xWasPressed = false;

  private static boolean isNotClimbing() {
    return !(m_state == State.Climbing);
  }

  private double startShootingRotorPosition = 0;

  enum State {
    Idle,
    Shooting,
    Climbing
  }

  public StateMachine(
      DriveSubsystem drivetrain,
      Shooter shooter,
      Intake intake,
      DyeRotor dyeRotor,
      Climb climb,
      CommandXboxController driverController) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
    m_dyeRotor = dyeRotor;
    m_climb = climb;

    m_publisher = new Publisher(m_drivetrain, m_shooter, m_intake, m_dyeRotor, m_climb);
    m_driverHID = driverController.getHID();

    if (Robot.isSimulation()) {
      // Simulate fuel being shot out of robot
      // TODO: Adjust number of rotations it takes to index 1 ball
      new Trigger(
              () ->
                  m_state == State.Shooting
                      && m_dyeRotor.getRotorPosition() - startShootingRotorPosition > 0.25)
          .onTrue(
              Commands.runOnce(
                  () -> {
                    LinearVelocity exitVelocity =
                        MetersPerSecond.of(m_shooter.getRawFlywheelTarget() / 6787 * 16.2);

                    Simulation.getInstance()
                        .shootFuel(
                            m_drivetrain
                                .getPose()
                                .getRotation()
                                .plus(Rotation2d.fromDegrees(m_shooter.getRawTurretTarget())),
                            exitVelocity,
                            Degrees.of(m_shooter.getRawHoodTarget()));
                    startShootingRotorPosition = m_dyeRotor.getRotorPosition();
                  }));

      // Simulate the time time it takes for fuel to get from dye rotor to shooter
      new Trigger(
              () ->
                  m_state != State.Shooting
                      && m_dyeRotor.isRunning()
                      && Simulation.getInstance().getFuelCount() != 0)
          .onTrue(
              Commands.waitSeconds(0.25)
                  .andThen(() -> Simulation.getInstance().setPreloaded(true)));
    }
  }

  /**
   * Sets up triggers that pause/resume the dye rotor based on flywheel readiness during shooting.
   */
  public void configureBindings() {
    Trigger pauseShooter =
        new Trigger(() -> m_state == State.Shooting && !m_shooter.readyToShoot());
    pauseShooter.onTrue(Commands.runOnce(() -> m_dyeRotor.pause()));

    Trigger resumeShooter =
        new Trigger(() -> m_state == State.Shooting && m_shooter.readyToShoot());
    resumeShooter.onTrue(Commands.runOnce(() -> m_dyeRotor.resume()));

    Trigger xNewPress =
        new Trigger(
            () -> {
              boolean current = m_driverHID.getXButton();
              if (current && !xWasPressed) {
                xWasPressed = true;
                return true;
              }
              if (!current) xWasPressed = false;
              return false;
            });

    Trigger stopPreload =
        new Trigger(() -> m_state != State.Shooting && m_dyeRotor.isRunning()).and(xNewPress);
    stopPreload.onTrue(m_dyeRotor.stop());

    Trigger startPreload =
        new Trigger(() -> m_state != State.Shooting && !m_dyeRotor.isRunning()).and(xNewPress);
    startPreload.onTrue(this.preloadCommand());

    m_shooter.configureShooterBindings();
  }

  public boolean phaseShift() {
    return phaseShiftActive;
  }

  public boolean phaseShiftWarning() {
    return phaseShiftWarningActive;
  }

  private boolean isPhaseShiftTime(double matchTime) {
    int wholeSeconds = (int) Math.round(matchTime);
    return wholeSeconds == 130 || wholeSeconds == 105 || wholeSeconds == 80 || wholeSeconds == 55;
  }

  private boolean isPhaseShiftWarningTime(double matchTime) {
    int wholeSeconds = (int) Math.round(matchTime);
    return wholeSeconds == 138 || wholeSeconds == 113 || wholeSeconds == 88 || wholeSeconds == 63;
  }

  /** Spins up flywheel, preloads fuel, then fires. Only runs from Idle. */
  public Command shoot() {
    // Run preload (dye rotor until fuel loaded, then stop) in parallel with
    // startShooter (spin flywheel until at setpoint). If startShooter finishes
    // first, just run the
    // dye rotor immediately.
    return preload()
        .withDeadline(m_shooter.startShooter().until(() -> m_shooter.readyToShoot()))
        .andThen(
            m_shooter
                .startShooter()
                .alongWith(m_dyeRotor.start())
                .beforeStarting(
                    () -> {
                      startShootingRotorPosition = m_dyeRotor.getRotorPosition();
                      setState(State.Shooting);
                      m_drivetrain.setShootingStateTrue();
                    }))
        .finallyDo(
            () -> {
              m_drivetrain.setShootingStateFalse();
              CommandScheduler.getInstance().schedule(stopShoot());
            })
        .onlyIf(() -> m_state == State.Idle);
  }

  /** Stops the flywheel and dye rotor, returns to Idle. */
  public Command stopShoot() {
    return m_shooter
        .stopShooter()
        .alongWith(m_dyeRotor.stop())
        .withName("stop shooting")
        .beforeStarting(
            () -> {
              m_state = State.Idle;
            });
  }

  /** Schedules a preload if not currently shooting. Safe to call anytime. */
  public Command preloadCommand() {
    return Commands.runOnce(
        () -> {
          if (m_state == State.Shooting) return;
          CommandScheduler.getInstance().schedule(preload());
        });
  }

  /** Stows the intake then deploys the climbing mechanism. */
  public Command deployClimber() {
    return Commands.sequence(
        m_shooter.stowTurretCommand(),
        m_intake
            .stow()
            .repeatedly() // TODO - this is hacky since stow is a runOnce
            .until(() -> m_intake.atSetpoint())
            .andThen(m_climb.deploy()));
  }

  /** Deploys climber and climbs. Only runs from Idle. */
  public Command climb() {
    return deployClimber()
        .until((() -> m_climb.atSetpoint()))
        .andThen(m_climb.climb())
        .onlyIf(() -> m_state == State.Idle || m_state == State.Climbing)
        .beforeStarting(() -> setState(State.Climbing));
  }

  /** Reverses the climb and returns to Idle. Only runs from Climbing. */
  public Command unclimb() {
    return m_climb
        .unclimb()
        .until(() -> m_climb.atSetpoint())
        .onlyIf(() -> m_state == State.Climbing)
        .beforeStarting(() -> m_shooter.clearTurretOverride())
        .andThen(() -> setState(State.Idle));
  }

  /** Runs the dye rotor until fuel is loaded, then stops. */
  public Command preload() {
    return m_dyeRotor
        .start()
        .until(() -> m_shooter.getFuelLimitSwitch())
        .andThen(m_dyeRotor.stop())
        .withName("preload");
  }

  /** Runs the intake. Blocked while climbing. */
  public Command intakeSequence() {
    return (m_intake.intake().onlyIf(StateMachine::isNotClimbing));
  }

  /** Reverses the intake. Blocked while climbing. */
  public Command extakeSequence() {
    return (m_intake.extake().onlyIf(StateMachine::isNotClimbing));
  }

  /** Stows the intake. Blocked while climbing. */
  public Command stowSequence() {
    return (m_intake.stow().onlyIf(StateMachine::isNotClimbing));
  }

  // Auto commands

  /**
   * Starts shooting for auto. Has zero subsystem requirements so it won't cancel the auto's
   * SequentialCommandGroup — sets flags directly instead.
   */
  public Command startShootingAuto() {
    return Commands.runOnce(
        () -> {
          startShootingRotorPosition = m_dyeRotor.getRotorPosition();
          setState(State.Shooting);
          m_drivetrain.setShootingStateTrue();
          m_shooter.setIsShooting(true);
          m_dyeRotor.startDirect();
        });
  }

  /** Stops shooting for auto. Zero subsystem requirements, same as {@link #startShootingAuto()}. */
  public Command stopShootAuto() {
    return Commands.runOnce(
        () -> {
          m_state = State.Idle;
          m_drivetrain.setShootingStateFalse();
          m_shooter.setIsShooting(false);
          m_dyeRotor.stopDirect();
        });
  }

  /**
   * @param timeout max seconds to run intake
   */
  public Command intakeSequenceAuto(double timeout) {
    return m_intake.intake().onlyIf(StateMachine::isNotClimbing).withTimeout(timeout);
  }

  /**
   * @param timeout max seconds to run extake
   */
  public Command extakeSequenceAuto(double timeout) {
    return m_intake.extake().onlyIf(StateMachine::isNotClimbing).withTimeout(timeout);
  }

  /**
   * @param timeout max seconds to run stow
   */
  public Command stowSequenceAuto(double timeout) {
    return m_intake.stow().onlyIf(StateMachine::isNotClimbing).withTimeout(timeout);
  }

  /**
   * Runs shooter and dye rotor for a duration, then stops. Only runs if already in Shooting state.
   *
   * @param shootTimeout max seconds to shoot before stopping
   */
  public Command waitForScore(double shootTimeout) {
    return m_shooter
        .startShooter()
        .alongWith(m_dyeRotor.start())
        .withTimeout(shootTimeout)
        .andThen(stopShootAuto())
        .onlyIf(() -> m_state == State.Shooting);
  }

  /** Returns the current robot state. */
  public State getState() {
    return m_state;
  }

  /** Sets the current robot state. */
  public void setState(State state) {
    m_state = state;
  }

  private void runTargeting() {
    Translation2d robotPosition = m_drivetrain.getPose().getTranslation();
    Rotation2d robotHeading = m_drivetrain.getPose().getRotation();

    if (m_drivetrain.isInAllianceZone()) {

      m_shooter.calculate(
          robotPosition,
          robotHeading,
          m_drivetrain.getFieldRelativeVelocity(),
          Field.getAllianceHub().toTranslation2d(),
          ShooterConstants.kLatencyCompensation);
      return;
    }

    double airstrikeX = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/x", 0));
    double airstrikeY = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/y", 0));

    if (airstrikeX == 0 && airstrikeY == 0) {
      return;
    }

    Translation2d airstrikeTarget = new Translation2d(airstrikeX, airstrikeY);

    m_shooter.calculate(
        robotPosition,
        robotHeading,
        m_drivetrain.getFieldRelativeVelocity(),
        airstrikeTarget,
        ShooterConstants.kLatencyCompensation);
  }

  @Override
  public void periodic() {
    runTargeting();
    phaseShiftActive = isPhaseShiftTime(DriverStation.getMatchTime());
    phaseShiftWarningActive = isPhaseShiftWarningTime(DriverStation.getMatchTime());

    SmartDashboard.putString(
        "State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    SmartDashboard.putString("State Machine/State", m_state.toString());

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("State Machine/Phase Shift Active", phaseShiftActive);

    m_publisher.publish();
  }
}
