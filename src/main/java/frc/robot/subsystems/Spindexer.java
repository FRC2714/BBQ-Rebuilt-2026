// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */


  private SparkFlex rotorMotor = new SparkFlex(Constants.SpindexerConstants.kRotorMotorCanId, MotorType.kBrushless);
  private SparkFlex feederMotor = new SparkFlex(Constants.SpindexerConstants.kFeederMotorCanId, MotorType.kBrushless);

  

  public Spindexer() {
    // Configs for rotorMotor
    rotorMotor.configure(
        Configs.Intake.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Configs for feederMotor
    feederMotor.configure(
        Configs.Intake.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
