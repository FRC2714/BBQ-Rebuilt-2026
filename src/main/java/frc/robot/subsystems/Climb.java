// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private final SparkFlex leftMotor =
      new SparkFlex(Constants.ClimbConstants.kLeftMotorCanID, MotorType.kBrushless);
  private final SparkFlex rightMotor =
      new SparkFlex(Constants.ClimbConstants.kRightMotorCanID, MotorType.kBrushless);
  private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

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

  private void extend() {
    leftController.setSetpoint(ClimbConstants.kExtendSetpoint, ControlType.kPosition);
  }

  private void retract() {
    leftController.setSetpoint(ClimbConstants.kRetractSetpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {}
}
