package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.DriveConstants;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final DyeRotor m_dyeRotor;
  private final Publisher m_publisher;
  

  private static State m_state = State.Idle;

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
      DriveSubsystem drivetrain, Shooter shooter, Intake intake, DyeRotor dyeRotor) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
    m_dyeRotor = dyeRotor;

    m_publisher = new Publisher(m_drivetrain, m_shooter, m_intake, m_dyeRotor);

    if (Robot.isSimulation()) {
      // Simulate fuel being shot out of robot
      // TODO: Adjust number of rotations it takes to index 1 ball
      new Trigger(
              () ->
                  m_state == State.Shooting
                      && m_dyeRotor.getRotorPosition() - startShootingRotorPosition > 0.5)
          .onTrue(
              Commands.runOnce(
                  () -> {
                    LinearVelocity exitVelocity =
                        MetersPerSecond.of(
                            Units.rotationsPerMinuteToRadiansPerSecond(m_shooter.getFlywheelSpeed())
                                * Units.inchesToMeters(1.5));

                    Simulation.getInstance()
                        .shootFuel(
                            m_drivetrain
                                .getPose()
                                .getRotation()
                                .plus(Rotation2d.fromDegrees(m_shooter.getTurretPosition())),
                            exitVelocity,
                            Degrees.of(m_shooter.getHoodAngle()));
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

  public void configureBindings() {
    Trigger pauseShooter =
        new Trigger(() -> m_state == State.Shooting && !m_shooter.readyToShoot());
    pauseShooter.onTrue(Commands.runOnce(() -> m_dyeRotor.pause()));

    Trigger resumeShooter =
        new Trigger(() -> m_state == State.Shooting && m_shooter.readyToShoot());
    resumeShooter.onTrue(Commands.runOnce(() -> m_dyeRotor.resume()));
  }

  public Command preloadCommand() {
    return Commands.runOnce(
        () -> {
          if (m_state == State.Shooting) return;

          CommandScheduler.getInstance().schedule(preload());
        });
  }

  public Command preload() {
    return m_dyeRotor
        .start()
        .until(() -> m_shooter.getFuelLimitSwitch())
        .andThen(m_dyeRotor.stop())
        .withName("preload");
  }

  public Command shoot() {
    // Run preload (dye rotor until fuel loaded, then stop) in parallel with
    // startShooter (spin flywheel until at setpoint). If startShooter finishes first, just run the
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
            });
  }

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

  public State getState() {
    return m_state;
  }

  public void setState(State state) {
    m_state = state;
  }

  // Generalization of updating the targets
  private static final double kLatencyCompensation = 0.1;
  private static final double kBaselineHorizontalVelocity = 6.0;

  private void aimAt(Translation2d target, Translation2d robotPosition, Rotation2d robotHeading) {
    Translation2d robotVelocity = m_drivetrain.getFieldRelativeVelocity();

    Translation2d futurePosition = robotPosition.plus(robotVelocity.times(kLatencyCompensation));

    Translation2d toGoal = target.minus(futurePosition);
    Translation2d targetDirection = toGoal.div(toGoal.getNorm());
    Translation2d targetVelocity = targetDirection.times(kBaselineHorizontalVelocity);
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    double distanceToTarget = toGoal.getNorm();

    Translation2d virtualTarget =
        futurePosition.plus(shotVelocity.div(shotVelocity.getNorm()).times(distanceToTarget));

    Translation2d robotToTarget = virtualTarget.minus(robotPosition);
    Rotation2d fieldAngle = robotToTarget.getAngle();
    Rotation2d turretAngle = fieldAngle.minus(robotHeading);

    m_shooter.updateTurretTarget(turretAngle.getDegrees());
    m_shooter.updateHoodTarget(distanceToTarget);
    m_shooter.updateFlywheelTarget(distanceToTarget);
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

  private void runTargeting() {

    Translation2d robotPosition = m_drivetrain.getPose().getTranslation();
    Rotation2d robotHeading = m_drivetrain.getPose().getRotation();

    if (m_drivetrain.isInAllianceZone()) {
      aimAt(Field.getAllianceHub().toTranslation2d(), robotPosition, robotHeading);
      return;
    }

    double airstrikeX = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/x", 0));
    double airstrikeY = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/y", 0));

    if (airstrikeX == 0 && airstrikeY == 0) {
      m_shooter.updateTurretTarget(0.0);
      return;
    }

    Translation2d airstrikeTarget = new Translation2d(airstrikeX, airstrikeY);
    aimAt(airstrikeTarget, robotPosition, robotHeading);
  }

  @Override
  public void periodic() {
    runTargeting();

    SmartDashboard.putString(
        "State Machine/Current Comamand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
    SmartDashboard.putString("State Machine/State", m_state.toString());
    SmartDashboard.putNumber("xSpeedSim", m_drivetrain.getXSpeed());
    SmartDashboard.putNumber("ySpeedSim", m_drivetrain.getYSpeed());

    m_publisher.publish();
  }
}
