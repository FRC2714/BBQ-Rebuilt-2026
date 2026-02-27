package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.HoodSetpoints;
import frc.robot.Constants.ShooterConstants.TurretSetpoints;
import frc.robot.Robot;
import frc.robot.Simulation;

public class Shooter extends SubsystemBase {
  private SparkFlex turretMotor =
      new SparkFlex(Constants.ShooterConstants.kTurretCanId, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();

  private RelativeEncoder turretRelativeEncoder = turretMotor.getExternalEncoder();

  private SparkFlex hoodMotor =
      new SparkFlex(Constants.ShooterConstants.kHoodCanId, MotorType.kBrushless);
  private SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
  private RelativeEncoder hoodRelativeEncoder = hoodMotor.getExternalEncoder();

  private SparkFlex flywheelMotorLeader =
      new SparkFlex(Constants.ShooterConstants.kFlywheelLeaderMotorId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController =
      flywheelMotorLeader.getClosedLoopController();
  private RelativeEncoder flywheelRelativeEncoder = flywheelMotorLeader.getEncoder();

  private SparkFlex flywheelMotorFollower =
      new SparkFlex(Constants.ShooterConstants.kFlywheelFollowerMotorId, MotorType.kBrushless);

  private SparkLimitSwitch fuelBeamBreak =
      flywheelMotorLeader.getForwardLimitSwitch(); // Placeholder for actual beam break sensor

  private double simFlywheelVelocity = 0.0;
  private double simHoodPosition = 0.0;

  private SparkLimitSwitch turretForwardLimitSwitch = turretMotor.getForwardLimitSwitch();
  private SparkLimitSwitch turretReverseLimitSwitch = turretMotor.getReverseLimitSwitch();

  private double turretCurrentTarget = TurretSetpoints.kStow;
  private double hoodCurrentTarget = HoodSetpoints.kStow;
  private double flywheelCurrentTarget = FlywheelSetpoints.kStow;

  public boolean wasZeroed = false;
  public boolean turretUpdated = false;

  private boolean isShooting = false;

  public record ShooterParams(double rpm, double hoodAngle, double timeOfFlight) {
    public static ShooterParams interpolate(ShooterParams a, ShooterParams b, double t) {
      double rpm = a.rpm + (b.rpm - a.rpm) * t;
      double hoodAngle = a.hoodAngle + (b.hoodAngle - a.hoodAngle) * t;
      double timeOfFlight = a.timeOfFlight + (b.timeOfFlight - a.timeOfFlight) * t;
      return new ShooterParams(rpm, hoodAngle, timeOfFlight);
    }
  }

  public class ShooterCommand {
    public final Rotation2d turretAngle;
    public final double rpm;
    public final double hoodAngle;

    public ShooterCommand(Rotation2d turretAngle, double rpm, double hoodAngle) {
      this.turretAngle = turretAngle;
      this.rpm = rpm;
      this.hoodAngle = hoodAngle;
    }
  }

  private static final InterpolatingTreeMap<Double, ShooterParams> shooterMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParams::interpolate);

  static {
    shooterMap.put(1.2, new ShooterParams(2650.0, 72.276537, 0.94));
    shooterMap.put(2.0, new ShooterParams(2714.0, 67.276537, 0.95));
    shooterMap.put(3.0, new ShooterParams(3050.0, 64.276537, 1.1));
    shooterMap.put(4.0, new ShooterParams(3450.0, 62.276537, 1.25));
    shooterMap.put(5.0, new ShooterParams(3800.0, 60.276537, 1.34));
    shooterMap.put(6.0, new ShooterParams(4275.0, 58.276537, 1.47));
    shooterMap.put(7.0, new ShooterParams(4800.0, 54.276537, 1.48));
    shooterMap.put(8.0, new ShooterParams(5750.0, 54.276537, 1.64));
    shooterMap.put(8.5, new ShooterParams(6300.0, 54.276537, 1.64));
  }

  /**
   * Courtesy of https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2 and
   * https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/dynamic-shooting.html
   *
   * @param robotPosition
   * @param robotHeading
   * @param robotVelocity
   * @param goalPosition
   * @param latencyCompensation
   */
  public void calculate(
      Translation2d robotPosition,
      Rotation2d robotHeading,
      Translation2d robotVelocity,
      Translation2d goalPosition,
      double latencyCompensation) {

    // 1. Project future position using latency compensation
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));

    // Relative position and velocity of the target from the robot's perspective.
    // The target (goal) is stationary, so its velocity relative to the robot is
    // simply the negation of the robot's velocity.
    Translation2d relativePosition = goalPosition.minus(futurePos);
    Translation2d relativeVelocity = robotVelocity.times(-1);

    // 2-4. Iteratively refine the shot using time-of-flight.
    // We begin with the raw distance to the target, then on each iteration we
    // predict where the target will be when the gamepiece arrives and re-look-up
    // the TOF for that adjusted position. We stop when TOF has converged.
    double timeOfFlight = 0.0;
    Translation2d adjustedRelativePosition = relativePosition;

    final int MAX_ITERATIONS = 10;
    final double CONVERGENCE_THRESHOLD = 0.001; // seconds

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double distance = adjustedRelativePosition.getNorm();
      ShooterParams params = shooterMap.get(distance);
      double newTimeOfFlight = params.timeOfFlight;

      // Check convergence before updating so we exit with the stable TOF value
      if (Math.abs(newTimeOfFlight - timeOfFlight) < CONVERGENCE_THRESHOLD) {
        timeOfFlight = newTimeOfFlight;
        break;
      }

      timeOfFlight = newTimeOfFlight;

      // Predict where the target will be (relative to the robot) when the
      // gamepiece arrives: shift the relative position by relative velocity * TOF
      adjustedRelativePosition = relativePosition.plus(relativeVelocity.times(timeOfFlight));
    }

    // 5. Once converged, look up control variables for the adjusted position
    double adjustedDistance = adjustedRelativePosition.getNorm();
    ShooterParams adjustedParams = shooterMap.get(adjustedDistance);

    // Aim toward the predicted future position of the target rather than its
    // current position, accounting for relative motion over the time of flight
    Rotation2d turretAngle = adjustedRelativePosition.getAngle();
    double requiredRpm = adjustedParams.rpm;
    double requiredHoodAngle = adjustedParams.hoodAngle;

    // 6. Set outputs
    this.flywheelCurrentTarget = requiredRpm;
    this.hoodCurrentTarget = requiredHoodAngle;
    this.turretCurrentTarget =
        turretAngle.relativeTo(robotHeading).getDegrees();
  }

  private Debouncer flywheelDebouncer =
      new Debouncer(ShooterConstants.kFlywheelDebounceTimeSeconds, DebounceType.kFalling);
  private Debouncer turretDebouncer =
      new Debouncer(ShooterConstants.kTurretDebounceTimeSeconds, DebounceType.kFalling);
  private Debouncer hoodDebouncer =
      new Debouncer(ShooterConstants.kHoodDebounceTimeSeconds, DebounceType.kFalling);
  // Simulation
  DCMotor flywheelMotorSim = DCMotor.getNeoVortex(2);
  SparkFlexSim flywheelSparkSim = new SparkFlexSim(flywheelMotorLeader, flywheelMotorSim);
  FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(flywheelMotorSim, 0.001, 1), flywheelMotorSim);

  DCMotor turretMotorSim = DCMotor.getNeo550(1);
  SparkFlexSim turretSparkSim = new SparkFlexSim(turretMotor, turretMotorSim);
  LinearSystemSim<N2, N1, N2> turretSim =
      new LinearSystemSim<>(
          LinearSystemId.createDCMotorSystem(
              turretMotorSim, ShooterConstants.kTurretMOI, ShooterConstants.kTurretGearRatio));

  DCMotor hoodMotorSim = DCMotor.getNeo550(1);
  SparkFlexSim hoodSparkSim = new SparkFlexSim(hoodMotor, hoodMotorSim);
  LinearSystemSim<N2, N1, N2> hoodSim =
      new LinearSystemSim<>(LinearSystemId.createDCMotorSystem(hoodMotorSim, 0.001, 10));

  public Shooter() {
    turretMotor.configure(
        Configs.Shooter.turretConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodMotor.configure(
        Configs.Shooter.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelMotorLeader.configure(
        Configs.Shooter.flywheelConfigLeader,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    flywheelMotorFollower.configure(
        Configs.Shooter.flywheelConfigFollower,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean getFuelLimitSwitch() {
    if (Robot.isSimulation()) {
      return Simulation.getInstance().isPreloaded();
    }
    return fuelBeamBreak.isPressed();
  }

  public double getTurretPosition() {
    return turretRelativeEncoder.getPosition();
  }

  private double normalizeTurretTarget(double angleDegrees) {
    double min = ShooterConstants.kTurretMinRange;
    double max = ShooterConstants.kTurretMaxRange;

    // Fold into one revolution first so we can generate equivalent candidates.
    double base = angleDegrees % 360.0;
    if (base > 180.0) {
      base -= 360.0;
    } else if (base <= -180.0) {
      base += 360.0;
    }

    double currentPosition = turretRelativeEncoder.getPosition();
    double bestTarget = Double.NaN;
    double bestError = Double.POSITIVE_INFINITY;

    for (int k = -2; k <= 2; k++) {
      double candidate = base + (k * 360.0);
      if (candidate < min || candidate > max) {
        continue;
      }

      double error = Math.abs(candidate - currentPosition);
      if (error < bestError) {
        bestError = error;
        bestTarget = candidate;
      }
    }

    if (!Double.isNaN(bestTarget)) {
      return bestTarget;
    }

    return Math.max(min, Math.min(max, base));
  }

  public double getHoodPosition() {
    return hoodRelativeEncoder.getPosition();
  }

  public void updateShootCommand(ShooterCommand command) {
    turretCurrentTarget = normalizeTurretTarget(command.turretAngle.getDegrees());
    flywheelCurrentTarget = command.rpm;
    hoodCurrentTarget = command.hoodAngle;
  }

  public double getFlywheelSpeed() {
    return flywheelRelativeEncoder.getVelocity();
  }

  public SparkFlex getTurretMotor() {
    return this.turretMotor;
  }

  public void setHoodAngle(double angle) {
    hoodCurrentTarget = angle;
  }

  public double getHoodAngle() {
    return hoodCurrentTarget;
  }

  public Command startShooter() {
    return this.run(
        () -> {
          isShooting = true;
        });
  }

  public Command stopShooter() {
    return this.run(
        () -> {
          isShooting = false;
        });
  }

  public boolean readyToShoot() {
    return flywheelAtSetpoint() && turretAtSetpoint() && hoodAtSetpoint();
  }

  public boolean flywheelAtSetpoint() {
    boolean atSetpoint =
        Math.abs(flywheelRelativeEncoder.getVelocity() - flywheelCurrentTarget) < 100;
    return flywheelDebouncer.calculate(atSetpoint);
  }

  public boolean turretAtSetpoint() {
    boolean atSetpoint = Math.abs(turretRelativeEncoder.getPosition() - turretCurrentTarget) < 5;
    return turretDebouncer.calculate(atSetpoint);
  }

  public boolean hoodAtSetpoint() {
    boolean atSetpoint = Math.abs(hoodRelativeEncoder.getPosition() - hoodCurrentTarget) < 2;
    return hoodDebouncer.calculate(atSetpoint);
  }

  public void zeroTurret() {
    if (!wasZeroed && turretMotor.getForwardLimitSwitch().isPressed()) {
      wasZeroed = true;
      turretRelativeEncoder.setPosition(ShooterConstants.kTurretMaxRange);
    } else if (!wasZeroed && turretMotor.getReverseLimitSwitch().isPressed()) {
      wasZeroed = true;
      turretRelativeEncoder.setPosition(ShooterConstants.kTurretMinRange);
    } else if (!turretMotor.getReverseLimitSwitch().isPressed()
        && !turretMotor.getForwardLimitSwitch().isPressed()) {
      wasZeroed = false;
    }
  }

  public Command zeroTurretSequence() {
    if (!wasZeroed) {
      wasZeroed = true;
      return new RunCommand(
              () -> {
                turretMotor.set(1);
              },
              this)
          .until(
              () ->
                  turretMotor.getForwardLimitSwitch().isPressed()
                      || turretMotor.getReverseLimitSwitch().isPressed())
          .andThen(new InstantCommand(() -> turretMotor.set(0), this));
    } else {
      return new InstantCommand();
    }
  }

  public void disableLimitSwitchAutoZeroing() {
    turretUpdated = true;
    SparkFlexConfig disableLimitSwitchZeroingConfig = new SparkFlexConfig();
    disableLimitSwitchZeroingConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(
        Behavior.kKeepMovingMotor);
    disableLimitSwitchZeroingConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(
        Behavior.kKeepMovingMotor);
    turretMotor.configure(
        disableLimitSwitchZeroingConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void setTurretAngle(double angle) {
    turretRelativeEncoder.setPosition(angle);
  }

  public void configureShooterBindings() {
    Trigger disableLimitSwitch =
        new Trigger(
            () ->
                turretMotor.getForwardLimitSwitch().isPressed()
                    || turretMotor.getReverseLimitSwitch().isPressed());
    disableLimitSwitch.onTrue(
        Commands.runOnce(() -> disableLimitSwitchAutoZeroing()).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    turretCurrentTarget = normalizeTurretTarget(turretCurrentTarget);
    turretController.setSetpoint(turretCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    hoodController.setSetpoint(hoodCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    flywheelController.setSetpoint(
        isShooting ? flywheelCurrentTarget : 0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("Shooter/Flywheel/Expected Speed", flywheelCurrentTarget);
    SmartDashboard.putNumber(
        "Shooter/Flywheel/Actual Speed", flywheelRelativeEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter/Flywheel/At Setpoint", flywheelAtSetpoint());

    SmartDashboard.putNumber("Shooter/Turret/Setpoint", turretCurrentTarget);
    SmartDashboard.putNumber("Shooter/Turret/Position", turretRelativeEncoder.getPosition());
    SmartDashboard.putBoolean("Shooter/Turret/At Setpoint", turretAtSetpoint());

    SmartDashboard.putNumber("Shooter/Hood/Setpoint", hoodCurrentTarget);
    SmartDashboard.putNumber("Shooter/Hood/Position", hoodRelativeEncoder.getPosition());
    SmartDashboard.putBoolean("Shooter/Hood/At Setpoint", hoodAtSetpoint());

    SmartDashboard.putBoolean("Shooter/Ready To Shoot", readyToShoot());
    SmartDashboard.putBoolean("Shooter/Turret/wasZeroed", wasZeroed);
    SmartDashboard.putString(
        "Shooter/Turret/fwd limit switch behavior",
        turretMotor.configAccessor.limitSwitch.getForwardLimitSwitchTriggerBehavior().toString());
    SmartDashboard.putString(
        "Shooter/Turret/rev limit switch behavior",
        turretMotor.configAccessor.limitSwitch.getReverseLimitSwitchTriggerBehavior().toString());
    SmartDashboard.putBoolean(
        "Shooter/Turret/fwd limit switch pressed", turretMotor.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean(
        "Shooter/Turret/rev limit switch pressed", turretMotor.getReverseLimitSwitch().isPressed());

    SmartDashboard.putBoolean("turret updated", turretUpdated);
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInputVoltage(
        flywheelSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    flywheelSim.update(0.02);

    flywheelSparkSim.iterate(
        flywheelSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);

    simFlywheelVelocity += (flywheelCurrentTarget - simFlywheelVelocity) * 0.2;
    simHoodPosition += (hoodCurrentTarget - simHoodPosition) * 0.05;

    turretSim.setInput(turretSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    turretSim.update(0.02);

    turretSparkSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(turretSim.getOutput(1))
            * ShooterConstants.kTurretGearRatio
            * 2, // This is hack to make the turret reach the target faster in simulation
        RobotController.getBatteryVoltage(),
        0.02);

    hoodSim.setInput(hoodSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    hoodSim.update(0.02);
    hoodSparkSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(hoodSim.getOutput(1) * 10),
        RobotController.getBatteryVoltage(),
        0.02);
  }
}
