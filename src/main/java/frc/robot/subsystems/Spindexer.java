// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  // Rotor
  private SparkFlex rotorMotor =
      new SparkFlex(Constants.SpindexerConstants.kRotorMotorCanId, MotorType.kBrushless);

  // Feeder
  private SparkFlex feederMotor =
      new SparkFlex(Constants.SpindexerConstants.kFeederMotorCanId, MotorType.kBrushless);
  // Beam breaks
  private SparkLimitSwitch beamBreak1 = feederMotor.getForwardLimitSwitch();
  private SparkLimitSwitch beamBreak2 = feederMotor.getReverseLimitSwitch();
  Mechanism2d mech = new Mechanism2d(4, 4);
  MechanismRoot2d root = mech.getRoot("spindexer", 2, 2);


  private double feederCurrentTarget = 0;
  private double rotorCurrentTarget = 0;
  private boolean check = false;
  

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
    rotorCurrentTarget = power;
    rotorMotor.set(rotorCurrentTarget);
  }

  public void setFeederPower(double power) {
    feederCurrentTarget = power;
    feederMotor.set(power);
  }

  public Command loading() {
    return this.run(
        () -> {
          check = false;
          setRotorPower(Constants.SpindexerConstants.kRotorMotorPower);
          setFeederPower(Constants.SpindexerConstants.kFeederMotorPower);
        }).withName("loading");
  }

  public Command stop() {
    return this.run(
        () -> {
          setRotorPower(0);
          setFeederPower(0);
        });
  }

  public Command feedUntilFull() {
    return feedFullReady().andThen(
      this.run(() -> {
        setRotorPower(0);
        setFeederPower(0);
      }).withName("feedUntilFull")
    );
   
  }

  public Command feedFullReady()
  {
        return this.run(
            () -> {
              setRotorPower(Constants.SpindexerConstants.kRotorMotorPower + 20);
              setFeederPower(Constants.SpindexerConstants.kFeederMotorPower + 20);
            })
        .until(() -> check == true).withName("feedUntilFull");
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

  public boolean simPressTrue() {
    //System.out.println("Running SIM PRESS TRUE");
    //System.out.println("CHECK IS " + check);
    check = true;
    stop();
    return true;
  }

  public boolean simPressFalse() {
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rotor Power", rotorCurrentTarget);
    SmartDashboard.putNumber("Feeder Power", feederCurrentTarget);
    SmartDashboard.putBoolean("True", check);
    SmartDashboard.putString("Current Commnad", this.getCurrentCommand() != null ? this.getCurrentCommand().getName(): "None");
  }

}
