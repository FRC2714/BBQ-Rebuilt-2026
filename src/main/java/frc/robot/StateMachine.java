package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.Locale;
import java.util.Optional;

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
  private boolean disablePassing = true;

  private boolean xWasPressed = false;
  private static final int[] PHASE_SHIFT_TRANSITION_TIMES = {130, 105, 80, 55, 30};
  private static final double PHASE_SHIFT_WARNING_WINDOW_SECONDS = 5.0;

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

  public void pauseOrResumeShooter() {
    if (m_state == State.Shooting && !m_shooter.readyToShoot()) {
      m_dyeRotor.pause();
    } else if (m_state == State.Shooting && m_shooter.readyToShoot()) {
      m_dyeRotor.resume();
    }
  }

  /**
   * Sets up triggers that pause/resume the dye rotor based on flywheel readiness during shooting.
   */
  public void configureBindings() {
    // Trigger xNewPress =
    //     new Trigger(
    //         () -> {
    //           boolean current = m_driverHID.getXButton();
    //           if (current && !xWasPressed) {
    //             xWasPressed = true;
    //             return true;
    //           }
    //           if (!current) xWasPressed = false;
    //           return false;
    //         });

    // Trigger stopPreload =
    //     new Trigger(() -> m_state != State.Shooting && m_dyeRotor.isRunning()).and(xNewPress);
    // stopPreload.onTrue(m_dyeRotor.stop());

    // Trigger startPreload =
    //     new Trigger(() -> m_state != State.Shooting && !m_dyeRotor.isRunning()).and(xNewPress);
    // startPreload.onTrue(this.preloadCommand());

    Trigger agitate = new Trigger(() -> m_state == State.Shooting && !m_intake.isIntaking());
    agitate.whileTrue(m_intake.agitate());

    m_shooter.configureShooterBindings();
    m_intake.configureBindings();
  }

  public Command zeroPoseAuto() {
    return Commands.runOnce(
            () -> {
              if (Field.isRed()) {
                m_drivetrain.zeroPose(180);
                System.out.println("zeroed red");

              } else {
                m_drivetrain.zeroPose(0);
                System.out.println("zeroed blue");
              }
            })
        .ignoringDisable(true);
  }

  public boolean phaseShift() {
    return phaseShiftActive;
  }

  public boolean phaseShiftWarning() {
    return phaseShiftWarningActive;
  }

  private boolean isPhaseShiftWarningTime(double matchTime) {
    double timeUntilNextPhaseShift = getTimeUntilNextPhaseShift(matchTime);
    return timeUntilNextPhaseShift > 0
        && timeUntilNextPhaseShift <= PHASE_SHIFT_WARNING_WINDOW_SECONDS;
  }

  private double getTimeUntilNextPhaseShift(double matchTime) {
    if (!DriverStation.isTeleopEnabled()) {
      return -1;
    }

    for (int transitionTime : PHASE_SHIFT_TRANSITION_TIMES) {
      if (matchTime >= transitionTime) {
        return matchTime - transitionTime;
      }
    }
    return -1;
  }

  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      return true;
    } else if (matchTime > 105) {
      return shift1Active;
    } else if (matchTime > 80) {
      return !shift1Active;
    } else if (matchTime > 55) {
      return shift1Active;
    } else if (matchTime > 30) {
      return !shift1Active;
    } else {
      return true;
    }
  }

  private String formatTimeUntilNextPhaseShift(double matchTime) {
    return String.format(Locale.US, "%.2f", getTimeUntilNextPhaseShift(matchTime));
  }

  private boolean getPhaseShiftDashboardValue(double matchTime) {
    if (!isPhaseShiftWarningTime(matchTime)) {
      return phaseShiftActive;
    }
    return ((int) Math.floor(matchTime * 4.0)) % 2 == 0;
  }

  /** Spins up flywheel, preloads fuel, then fires. Only runs from Idle. */
  public Command shoot() {
    // Run preload (dye rotor until fuel loaded, then stop) in parallel with
    // startShooter (spin flywheel until at setpoint). If startShooter finishes
    // first, just run the
    // dye rotor immediately.
    return m_intake
        .extend()
        .andThen(
            m_shooter
                .startShooter()
                .until(() -> m_shooter.readyToShoot())
                .raceWith(Commands.waitSeconds(1.5)))
        .beforeStarting(
            () -> {
              m_shooter.clearTurretOverride();
              m_drivetrain.setShootingStateTrue();
            })
        .andThen(
            m_shooter
                .startShooter()
                .alongWith(m_dyeRotor.start())
                .beforeStarting(
                    () -> {
                      startShootingRotorPosition = m_dyeRotor.getRotorPosition();
                      setState(State.Shooting);
                    }))
        .onlyIf(() -> m_state == State.Idle);
  }

  public Command toggleOverride() {
    return Commands.runEnd(
            () -> m_shooter.setTurretOverride(0), () -> m_shooter.clearTurretOverride())
        .ignoringDisable(true);
  }

  public void enablePassing() {
    disablePassing = false;
  }

  public void disablePassing() {
    disablePassing = true;
  }

  /** Stops the flywheel and dye rotor, returns to Idle. */
  public Command stopShoot() {
    return m_shooter
        .stopShooter()
        .alongWith(m_dyeRotor.stop())
        .withName("stop shooting")
        .beforeStarting(
            () -> {
              m_drivetrain.setShootingStateFalse();
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
  // public Command deployClimber() {
  //   return Commands.sequence(
  //           m_shooter.stowTurretCommand(),
  //           m_intake.stow().andThen(Commands.waitUntil(m_intake::atSetpoint)),
  //           m_climb.deploy().until(m_climb::atSetpoint))
  //       .beforeStarting(() -> setState(State.Climbing));
  // }

  /** Deploys climber and climbs. Only runs from Idle. */
  // public Command climb() {
  //   return deployClimber()
  //       .andThen(m_climb.climb())
  //       .onlyIf(() -> m_state == State.Idle || m_state == State.Climbing);
  // }

  /** Reverses the climb and returns to Idle. Only runs from Climbing. */
  // public Command unclimb() {
  //   return m_climb
  //       .unclimb()
  //       .until(() -> m_climb.atSetpoint())
  //       .onlyIf(() -> m_state == State.Climbing)
  //       .andThen(() -> setState(State.Idle));
  // }

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
    return (m_intake
        .intake()
        .alongWith(
            Commands.waitUntil(
                    () ->
                        m_intake.getIntakePivotPosition()
                            < IntakeConstants.PivotConstants.kPivotClear)
                .andThen(() -> m_shooter.clearTurretOverride()))
        .onlyIf(StateMachine::isNotClimbing));
  }

  /** Reverses the intake. Blocked while climbing. */
  public Command extakeSequence() {
    return (m_intake.extake().onlyIf(StateMachine::isNotClimbing));
  }

  /** Stows the intake. Blocked while climbing. */
  public Command stowSequence() {
    return m_shooter
        .stowTurretCommand()
        .andThen((m_intake.stow().onlyIf(StateMachine::isNotClimbing)).alongWith(stopShoot()));
  }

  public Command extendIntakeSequence() {
    return m_intake.agitateOut().onlyIf(StateMachine::isNotClimbing);
  }

  public Command retractIntakeSequence() {
    return m_intake.agitateIn().onlyIf(StateMachine::isNotClimbing);
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

  public Command manualDyeRotor() {
    return Commands.runEnd(() -> m_dyeRotor.start(), () -> m_dyeRotor.stop());
  }

  public Command manualFlywheel() {
    return Commands.runEnd(() -> m_shooter.startShooter(), () -> m_shooter.stopShooter());
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

  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Auto Aim Target", Pose2d.struct).publish();

  private Translation2d getAutoAimTarget() {
    double robotY = m_drivetrain.getPose().getY();
    double centerY = FieldConstants.LinesHorizontal.center;

    if (Field.isRed()) {
      return robotY < centerY ? AutoAimConstants.kRedLeftTarget : AutoAimConstants.kRedRightTarget;
    } else {
      return robotY < centerY
          ? AutoAimConstants.kBlueRightTarget
          : AutoAimConstants.kBlueLeftTarget;
    }
  }

  public void runTargeting() {
    Translation2d robotPosition = m_drivetrain.getPose().getTranslation();
    Rotation2d robotHeading = m_drivetrain.getPose().getRotation();

    if (m_drivetrain.isInAllianceZone()) {
      m_shooter.calculate(
          robotPosition,
          robotHeading,
          m_drivetrain.getFieldRelativeVelocity(),
          m_drivetrain.getTurnRate(),
          Field.getAllianceHub().toTranslation2d(),
          ShooterConstants.kLatencyCompensation);
      return;
    }

    // Airstrike manual override takes priority
    double airstrikeX = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/x", 0));
    double airstrikeY = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/y", 0));
    boolean airstrikeHasTarget = SmartDashboard.getBoolean("airstrike/hasTarget", false);

    Translation2d target;
    if (airstrikeHasTarget) {
      target = new Translation2d(airstrikeX, airstrikeY);
    } else {
      target = getAutoAimTarget();
    }

    publisher.set(new Pose2d(target, new Rotation2d()));
    if (!disablePassing) {
      m_shooter.calculate(
          robotPosition,
          robotHeading,
          m_drivetrain.getFieldRelativeVelocity(),
          m_drivetrain.getTurnRate(),
          target,
          ShooterConstants.kLatencyCompensation);
    } else {
      m_shooter.setTurretAngle(0);
    }
  }

  @Override
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();
    phaseShiftActive = isHubActive();
    phaseShiftWarningActive = isPhaseShiftWarningTime(matchTime);

    SmartDashboard.putString(
        "State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    SmartDashboard.putString("State Machine/State", m_state.toString());

    SmartDashboard.putNumber("Match Time", matchTime);
    SmartDashboard.putBoolean(
        "State Machine/Phase Shift Active", getPhaseShiftDashboardValue(matchTime));
    SmartDashboard.putString(
        "State Machine/Time Until Next Phase Shift", formatTimeUntilNextPhaseShift(matchTime));

    m_publisher.publish();
  }
}
