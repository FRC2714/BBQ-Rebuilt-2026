// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.DyeRotorConstants;


public class DyeRotor extends SubsystemBase {
  
  //Hook
  //Roller
  private SparkFlex dyeRotorMotor = new SparkFlex(Constants.DyeRotorConstants.kDyeRotorMotorCanID, MotorType.kBrushless);
  
  private double dyeRotorCurrentTarget = 0;
  
  
  /** Creates a new Dyerotor. */
  public DyeRotor() {
    dyeRotorMotor.configure(
        Configs.DyeRotor.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public Command start()
  {
      return this.run(
        () -> {
          dyeRotorMotor.set(Constants.DyeRotorConstants.kDyeRotorPower);
        });
  }

  public Command stop()
  {
    return this.run(
      () -> {
        dyeRotorMotor.set(0);
      });
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

