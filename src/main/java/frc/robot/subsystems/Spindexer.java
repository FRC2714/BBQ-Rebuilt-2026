// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private SparkFlex rotorMotor =
      new SparkFlex(Constants.SpindexerConstants.kRotorMotorCanId, MotorType.kBrushless);

  private SparkFlex feederMotor =
      new SparkFlex(Constants.SpindexerConstants.kFeederMotorCanId, MotorType.kBrushless);
  private SparkLimitSwitch beamBreak1 = feederMotor.getForwardLimitSwitch();
  private SparkLimitSwitch beamBreak2 = feederMotor.getReverseLimitSwitch();

  public Spindexer() {
    // Configs for rotorMotor
    rotorMotor.configure(
        Configs.Spindexer.rotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // Configs for feederMotor
    feederMotor.configure(
        Configs.Spindexer.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setRotorPower(double power) {
    rotorMotor.set(power);
  }

  public void setFeederPower(double power) {
    feederMotor.set(power);
  }

  public Command loading() {
    return this.run(
      () -> {
        setRotorPower(Constants.SpindexerConstants.kRotorMotorPower);
        setFeederPower(Constants.SpindexerConstants.kFeederMotorPower);
      }
    );
  }

  public Command stop() {
    return this.run(
      () -> {
        setRotorPower(0);
        setFeederPower(0);
      }
    );
  }
  
  public Command feedUntilFull() {
    return this.run(
            () -> {
              setRotorPower(Constants.SpindexerConstants.kRotorMotorPower);
              setFeederPower(Constants.SpindexerConstants.kFeederMotorPower);
            })
        .until(() -> beamBreak1.isPressed())
        .andThen(
            this.run(
                () -> {
                  stop();
                }));
  }

  public Command reverseFuel() {
    return this.run(
            () -> {
              setFeederPower(-(Constants.SpindexerConstants.kRotorMotorPower));
            })
        .until(() -> !(beamBreak2.isPressed()))
        .andThen(
            this.run(
                () -> {
                  setFeederPower(0);
                }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
