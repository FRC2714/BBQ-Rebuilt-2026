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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.PivotSetpoints;

public class Shooter extends SubsystemBase {
  // turret motor and controller
  private SparkFlex turretMotor =
      new SparkFlex(Constants.TurretConstants.kTurretShooterCanId, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();
  private ArmFeedforward turretFF = new ArmFeedforward(0, TurretConstants.kG, getTurretPosition());
  private AbsoluteEncoder turretAbsoluteEncoder = turretMotor.getAbsoluteEncoder();

  // limit switch
  private SparkLimitSwitch turretLimitSwitch = turretMotor.getForwardLimitSwitch();

  private double turretCurrentTarget = PivotSetpoints.kStow;

  // Creates a flywheel
  public Shooter() {
    turretMotor.configure(
        Configs.Flywheel.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getTurretPosition() {
    return turretAbsoluteEncoder.getPosition();
  }

  public void updateTurretTarget(double updateValue) {
    turretCurrentTarget = updateValue;
  }

  // Runs every 20ms
  @Override
  public void periodic() {
    turretController.setSetpoint(
        turretCurrentTarget,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        turretFF.calculate(turretCurrentTarget, 0));
  }
}
