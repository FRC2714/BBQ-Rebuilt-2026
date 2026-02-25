// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private final SparkFlex leftMotor =
      new SparkFlex(Constants.ClimbConstants.kLeftMotorCanID, MotorType.kBrushless);
  private final SparkFlex rightMotor =
      new SparkFlex(Constants.ClimbConstants.kRightMotorCanID, MotorType.kBrushless);
  private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private final SparkClosedLoopController rightController = rightMotor.getClosedLoopController();

  private double setpoint = 0;

  /** Creates a new Climb. */
  public Climb() {
    leftMotor.configure(
        Configs.Climb.leftClimbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightMotor.configure(
        Configs.Climb.rightClimbConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public Command extend() {
    return this.run(
        () -> {
          leftMotor.set(Constants.ClimbConstants.kExtendSpeed);
          rightMotor.set(Constants.ClimbConstants.kExtendSpeed);
          moveToExtension();
        });
  }

  public Command retract() {
    return this.run(
        () -> {
          leftMotor.set(Constants.ClimbConstants.kRetractSpeed);
          rightMotor.set(Constants.ClimbConstants.kRetractSpeed);
          moveToRetract();
        });
  }

  private void moveToExtension() {
    setpoint = Constants.ClimbConstants.kExtendSetpoint;
    leftController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void moveToRetract() {
    setpoint = Constants.ClimbConstants.kRetractSetpoint;
    leftController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void periodic() {}
}
