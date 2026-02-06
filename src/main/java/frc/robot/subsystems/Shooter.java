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
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.HoodSetpoints;
import frc.robot.Constants.ShooterConstants.TurretSetpoints;
import frc.robot.utils.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
  // turret motors and controllers
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
  private RelativeEncoder flywheelRelativeEncoder = flywheelMotorLeader.getExternalEncoder();

  private SparkFlex flywheelMotorFollower =
      new SparkFlex(Constants.ShooterConstants.kFlywheelFollowerMotorId, MotorType.kBrushless);

  // Simple simulation state (used only when running in WPILib simulation)
  private double simFlywheelVelocity = 0.0; // RPM-ish
  private double simHoodPosition = 0.0; // degrees-ish

  // limit switches
  private SparkLimitSwitch turretForwardLimitSwitch = turretMotor.getForwardLimitSwitch();
  private SparkLimitSwitch turretReverseLimitSwitch = turretMotor.getReverseLimitSwitch();

  private double turretCurrentTarget = TurretSetpoints.kStow;

  private double hoodTarget = HoodSetpoints.kStow;

  private double flywheelTargetSpeed = FlywheelSetpoints.kStow;

  private InterpolatingTreeMap hoodAngleMap;
  private InterpolatingTreeMap flywheelSpeedMap;
  private DriveSubsystem m_DriveSubsystem;

  // Creates a hood
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
    // Keys: distance to hub in meters -> value: hood position (encoder units or degrees depending
    // on hood config)
    // These are example values for simulation; tune for your robot's hardware.
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
    // Keys: distance to hub in meters -> value: flywheel speed (in native closed-loop units, e.g.
    // RPM)
    // These are example values for simulation; tune for your robot's hardware and controller
    // config.
    flywheelSpeedMap.put(1.0, 1500.0);
    flywheelSpeedMap.put(2.0, 2200.0);
    flywheelSpeedMap.put(3.0, 2700.0);
    flywheelSpeedMap.put(4.0, 3200.0);
    flywheelSpeedMap.put(5.0, 3600.0);
    flywheelSpeedMap.put(6.0, 4000.0);
    flywheelSpeedMap.put(7.0, 4400.0);
    flywheelSpeedMap.put(8.0, 4800.0);
  }

  // Get positions
  public double getTurretPosition() {
    return turretRelativeEncoder.getPosition();
  }

  public double getHoodPosition() {
    return hoodRelativeEncoder.getPosition();
  }

  // Update the target positions
  public void updateTurretTarget(double updateValue) {
    turretCurrentTarget =
        MathUtil.clamp(
            updateValue,
            Constants.ShooterConstants.kTurretMinRange,
            Constants.ShooterConstants.kTurretMaxRange);
  }

  public double updateHoodTarget() {
    if (m_DriveSubsystem == null) {
      return HoodSetpoints.kStow;
    }
    Double dist = m_DriveSubsystem.getDistanceToHub();
    Double interpolated = hoodAngleMap.getInterpolated(dist);
    return interpolated == null ? HoodSetpoints.kStow : interpolated;
  }

  public void moveHoodToSetpoint() {
    hoodController.setSetpoint(updateHoodTarget(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void moveFlywheelToSetpoint() {
    flywheelController.setSetpoint(
        updateFlyWheelSpeedTarget(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public double updateFlyWheelSpeedTarget() {
    if (m_DriveSubsystem == null) {
      return FlywheelSetpoints.kStow;
    }
    Double dist = m_DriveSubsystem.getDistanceToHub();
    Double interpolated = flywheelSpeedMap.getInterpolated(dist);
    return interpolated == null ? FlywheelSetpoints.kStow : interpolated;
  }

  /** Provide DriveSubsystem reference so shooter can query vision-based distance. */
  public void setDriveSubsystem(DriveSubsystem ds) {
    m_DriveSubsystem = ds;
  }

  public void setFlywheelSpeed(double speed) {
    flywheelTargetSpeed = speed;
  }

  public double getFlywheelSpeed() {
    return flywheelController.getSetpoint();
  }

  public void setHoodAngle(double angle) {
    hoodTarget = angle;
  }

  public double getHoodAngle() {
    return hoodController.getSetpoint();
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

  // Runs every 20ms
  @Override
  public void periodic() {
    moveHoodToSetpoint();
    moveFlywheelToSetpoint();

    SmartDashboard.putNumber("Hood Angle", hoodController.getSetpoint());
    SmartDashboard.putNumber("Flywheel Speed", flywheelController.getSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    // The REV SparkFlex library may not provide simulated encoder values in this environment.
    // Provide a very small, deterministic simulation so AdvantageScope/SmartDashboard shows values.
    double targetFlywheel = flywheelController.getSetpoint();
    double targetHood = hoodController.getSetpoint();

    // Simple first-order response toward setpoint
    simFlywheelVelocity += (targetFlywheel - simFlywheelVelocity) * 0.2; // responsiveness tuning
    simHoodPosition += (targetHood - simHoodPosition) * 0.05;

    SmartDashboard.putNumber("Hood Angle", simHoodPosition);
    SmartDashboard.putNumber("Flywheel Speed", simFlywheelVelocity);
    SmartDashboard.putNumber("Turret Position", turretCurrentTarget);
  }
}
