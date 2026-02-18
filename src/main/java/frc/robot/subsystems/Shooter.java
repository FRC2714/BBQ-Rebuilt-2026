package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.HoodSetpoints;
import frc.robot.Constants.ShooterConstants.TurretSetpoints;
import frc.robot.Robot;
import frc.robot.utils.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
  private SparkFlex turretMotor =
      new SparkFlex(Constants.ShooterConstants.kTurretCanId, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();

  private RelativeEncoder turretRelativeEncoder = turretMotor.getExternalEncoder();
  private AbsoluteEncoder turretAbsoluteEncoder = turretMotor.getAbsoluteEncoder();

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
  private boolean fuelTrigger = false;
  private boolean isShooting = false;

  private InterpolatingTreeMap hoodAngleMap;
  private InterpolatingTreeMap flywheelSpeedMap;

  // Simulation
  DCMotor flywheelMotorSim = DCMotor.getNeoVortex(2);
  SparkFlexSim flywheelSparkSim = new SparkFlexSim(flywheelMotorLeader, flywheelMotorSim);
  FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(flywheelMotorSim, 0.001, 1), flywheelMotorSim);

  DCMotor turretMotorSim = DCMotor.getNEO(1);
  SparkFlexSim turretSparkSim = new SparkFlexSim(turretMotor, turretMotorSim);
  LinearSystemSim<N2, N1, N2> turretSim =
      new LinearSystemSim<>(LinearSystemId.createDCMotorSystem(turretMotorSim, 0.0001, 40));

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

    hoodAngleMap = new InterpolatingTreeMap();
    flywheelSpeedMap = new InterpolatingTreeMap();

    populateHoodAngleMap();
    populateFlywheelSpeedMap();
  }

  // CHANGE LATER
  public void populateHoodAngleMap() {
    hoodAngleMap.put(1.0, 5.0);
    hoodAngleMap.put(2.0, 8.0);
    hoodAngleMap.put(3.0, 12.0);
    hoodAngleMap.put(4.0, 18.0);
    hoodAngleMap.put(5.0, 26.0);
    hoodAngleMap.put(6.0, 34.0);
    hoodAngleMap.put(7.0, 42.0);
    hoodAngleMap.put(8.0, 50.0);
  }

  // CHANGE LATER
  public void populateFlywheelSpeedMap() {
    flywheelSpeedMap.put(1.0, 1500.0);
    flywheelSpeedMap.put(2.0, 2200.0);
    flywheelSpeedMap.put(3.0, 2700.0);
    flywheelSpeedMap.put(4.0, 3200.0);
    flywheelSpeedMap.put(5.0, 3600.0);
    flywheelSpeedMap.put(6.0, 4000.0);
    flywheelSpeedMap.put(7.0, 4400.0);
    flywheelSpeedMap.put(8.0, 4800.0);
  }

  public boolean getFuelLimitSwitch() {
    if (Robot.isSimulation()) {
      return fuelTrigger;
    }
    return fuelBeamBreak.isPressed();
  }

  public double getTurretPosition() {
    return turretAbsoluteEncoder.getPosition();
  }

  public double getHoodPosition() {
    return hoodRelativeEncoder.getPosition();
  }

  public void updateTurretTarget(double updateValue) {
    turretCurrentTarget =
        MathUtil.clamp(
            updateValue,
            Constants.ShooterConstants.kTurretMinRange,
            Constants.ShooterConstants.kTurretMaxRange);
  }

  public void updateHoodTarget(double distanceToHub) {
    Double interpolated = hoodAngleMap.getInterpolated(distanceToHub);
    hoodCurrentTarget = interpolated != null ? interpolated : HoodSetpoints.kStow;
  }

  public void updateFlywheelTarget(double distanceToHub) {
    Double interpolated = flywheelSpeedMap.getInterpolated(distanceToHub);
    flywheelCurrentTarget = interpolated != null ? interpolated : FlywheelSetpoints.kStow;
  }

  public double getFlywheelSpeed() {
    return flywheelCurrentTarget;
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

  // TODO: Debounce this
  public boolean flywheelAtSetpoint() {
    return Math.abs(flywheelRelativeEncoder.getVelocity() - flywheelCurrentTarget) < 100;
  }

  // TODO: Debounce this
  public boolean turretAtSetpoint() {
    return Math.abs(turretAbsoluteEncoder.getPosition() - turretCurrentTarget) < 5;
  }

  // TODO: Deboucne this
  public boolean hoodAtSetpoint() {
    return Math.abs(hoodRelativeEncoder.getPosition() - hoodCurrentTarget) < 2;
  }

  public void fuelTrue() {
    fuelTrigger = true;
  }

  public void fuelFalse() {
    fuelTrigger = false;
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

  @Override
  public void periodic() {
    turretController.setSetpoint(turretCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    hoodController.setSetpoint(hoodCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    flywheelController.setSetpoint(
        isShooting ? flywheelCurrentTarget : 0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("Shooter/Flywheel/Expected Speed", flywheelCurrentTarget);
    SmartDashboard.putNumber(
        "Shooter/Flywheel/Actual Speed", flywheelRelativeEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter/Flywheel/At Setpoint", flywheelAtSetpoint());

    SmartDashboard.putNumber("Shooter/Turret/Setpoint", turretCurrentTarget);
    SmartDashboard.putNumber("Shooter/Turret/Position", turretAbsoluteEncoder.getPosition());
    SmartDashboard.putBoolean("Shooter/Turret/At Setpoint", turretAtSetpoint());

    SmartDashboard.putNumber("Shooter/Hood/Setpoint", hoodCurrentTarget);
    SmartDashboard.putNumber("Shooter/Hood/Position", hoodRelativeEncoder.getPosition());
    SmartDashboard.putBoolean("Shooter/Hood/At Setpoint", hoodAtSetpoint());

    SmartDashboard.putBoolean("Shooter/Ready To Shoot", readyToShoot());
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
        Units.radiansPerSecondToRotationsPerMinute(turretSim.getOutput(1)),
        RobotController.getBatteryVoltage(),
        0.02);

    hoodSim.setInput(hoodSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    hoodSim.update(0.02);
    hoodSparkSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(hoodSim.getOutput(1) * 10),
        RobotController.getBatteryVoltage(),
        0.02);

    SmartDashboard.putNumber("Hood Angle", simHoodPosition);
    SmartDashboard.putNumber("Flywheel Speed", simFlywheelVelocity);
    SmartDashboard.putNumber("Turret Position", turretCurrentTarget);
    SmartDashboard.putBoolean("Fuel Loaded", fuelTrigger);

    zeroTurret();
  }
}
