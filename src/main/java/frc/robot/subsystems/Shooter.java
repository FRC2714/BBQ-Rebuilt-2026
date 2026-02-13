package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.HoodSetpoints;
import frc.robot.Constants.ShooterConstants.TurretSetpoints;
import frc.robot.utils.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
  private SparkFlex turretMotor = new SparkFlex(Constants.ShooterConstants.kTurretCanId, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();

  private RelativeEncoder turretRelativeEncoder = turretMotor.getExternalEncoder();

  private SparkFlex hoodMotor = new SparkFlex(Constants.ShooterConstants.kHoodCanId, MotorType.kBrushless);
  private SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
  private RelativeEncoder hoodRelativeEncoder = hoodMotor.getExternalEncoder();

  private SparkFlex flywheelMotorLeader = new SparkFlex(Constants.ShooterConstants.kFlywheelLeaderMotorId,
      MotorType.kBrushless);
  private SparkClosedLoopController flywheelController = flywheelMotorLeader.getClosedLoopController();
  private RelativeEncoder flywheelRelativeEncoder = flywheelMotorLeader.getExternalEncoder();

  private SparkFlex flywheelMotorFollower = new SparkFlex(Constants.ShooterConstants.kFlywheelFollowerMotorId,
      MotorType.kBrushless);

  private double simFlywheelVelocity = 0.0;
  private double simHoodPosition = 0.0;

  private SparkLimitSwitch turretForwardLimitSwitch = turretMotor.getForwardLimitSwitch();
  private SparkLimitSwitch turretReverseLimitSwitch = turretMotor.getReverseLimitSwitch();

  private double turretCurrentTarget = TurretSetpoints.kStow;
  private double hoodCurrentTarget = HoodSetpoints.kStow;
  private double flywheelCurrentTarget = FlywheelSetpoints.kStow;

  public boolean wasZeroed = false;

  private InterpolatingTreeMap hoodAngleMap;
  private InterpolatingTreeMap flywheelSpeedMap;

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

  public double getTurretPosition() {
    return turretRelativeEncoder.getPosition();
  }

  public double getHoodPosition() {
    return hoodRelativeEncoder.getPosition();
  }

  public void updateTurretTarget(double updateValue) {
    turretCurrentTarget = MathUtil.clamp(
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

  public void setFlywheelSpeed(double speed) {
    flywheelCurrentTarget = speed;
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
          setFlywheelSpeed(Constants.ShooterConstants.FlywheelSetpoints.kStartSpeed);
        });
  }

  public Command stopShooter() {
    return this.run(
        () -> {
          setFlywheelSpeed(0);
        });
  }

  public Command stowShooter() {
    return this.run(
        () -> {
          setHoodAngle(Constants.ShooterConstants.HoodSetpoints.kStow);
        });
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
    flywheelController.setSetpoint(flywheelCurrentTarget, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("Shooter/Hood Angle", hoodCurrentTarget);
    SmartDashboard.putNumber("Shooter/Flywheel Speed", flywheelCurrentTarget);
    SmartDashboard.putNumber("Shooter/Turret Target", turretCurrentTarget);
  }

  @Override
  public void simulationPeriodic() {
    simFlywheelVelocity += (flywheelCurrentTarget - simFlywheelVelocity) * 0.2;
    simHoodPosition += (hoodCurrentTarget - simHoodPosition) * 0.05;

    SmartDashboard.putNumber("Hood Angle", simHoodPosition);
    SmartDashboard.putNumber("Flywheel Speed", simFlywheelVelocity);
    SmartDashboard.putNumber("Turret Position", turretCurrentTarget);

    zeroTurret();
  }
}