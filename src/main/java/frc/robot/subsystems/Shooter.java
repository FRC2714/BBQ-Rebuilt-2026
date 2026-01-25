package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.TurretSetpoints;

public class Shooter extends SubsystemBase {
  // turret motors and controllers
  private SparkFlex turretMotor =
      new SparkFlex(Constants.ShooterConstants.kTurretCanId, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();
  private AbsoluteEncoder turretAbsoluteEncoder = turretMotor.getAbsoluteEncoder();

  // private SparkFlex hoodMotor =
  //     new SparkFlex(Constants.ShooterConstants.kHoodCanId, MotorType.kBrushless);
  // private SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
  // private AbsoluteEncoder hoodAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();

  // limit switches
  private SparkLimitSwitch turretLimitSwitch = turretMotor.getForwardLimitSwitch();
  // private SparkLimitSwitch hoodLimitSwitch = hoodMotor.getForwardLimitSwitch();

  private double turretCurrentTarget = TurretSetpoints.kStow;

  // private double hoodTarget = HoodSetpoints.kStow;

  // Creates a hood
  public Shooter() {
    turretMotor.configure(
        Configs.Shooter.turretConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // hoodMotor.configure(
    //     Configs.Shooter.hoodConfig,
    //     ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);
  }

  // Get positions
  public double getTurretPosition() {
    return turretAbsoluteEncoder.getPosition();
  }

  // public double getHoodPosition() {
  //   return hoodAbsoluteEncoder.getPosition();
  // }

  // Update the target positions
  public void updateTurretTarget(double updateValue) {
    turretCurrentTarget =
        MathUtil.clamp(
            updateValue,
            Constants.ShooterConstants.kTurretMinRange,
            Constants.ShooterConstants.kTurretMaxRange);
  }

  // public void updateHoodTarget(double updateValue) {
  //   hoodTarget =
  //       MathUtil.clamp(
  //           updateValue,
  //           Constants.ShooterConstants.kHoodMinRange,
  //           Constants.ShooterConstants.kHoodMaxRange);
  // }

  // Runs every 20ms
  @Override
  public void periodic() {
    turretController.setSetpoint(turretCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // hoodController.setSetpoint(hoodTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
