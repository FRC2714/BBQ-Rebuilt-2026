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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.DyeRotorConstants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;



public class DyeRotor extends SubsystemBase {
  
  
  private SparkFlex dyeRotorMotor = new SparkFlex(Constants.DyeRotorConstants.kDyeRotorMotorCanID, MotorType.kBrushless);
  private double dyeRotorCurrentTarget = 0;
  private double rotorAngleDeg = 0;

  
  
  /** Creates a new Dyerotor. */
  public DyeRotor() {
    dyeRotorMotor.configure(
        Configs.DyeRotor.dyeRotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        rotorArm.setAngle(45);
        SmartDashboard.putData("Dye Rotor Mech", mech2d);

  }

  public Command start()
  {
      return this.run(
        () -> {
          dyeRotorCurrentTarget = Constants.DyeRotorConstants.kDyeRotorPower;
          dyeRotorMotor.set(Constants.DyeRotorConstants.kDyeRotorPower);
        });
  }

  public Command stop()
  {
    return this.run(
      () -> {
        dyeRotorCurrentTarget = 0;
        dyeRotorMotor.set(0);
      });
  }

  //Mech2d for DyeRotor   
  private final Mechanism2d mech2d =
    new Mechanism2d(60, 60); // width, height in "virtual units"

  private final MechanismRoot2d rotorRoot =
    mech2d.getRoot("DyeRotorRoot", 29.5, 0);

  private final MechanismLigament2d rotorArm =
      rotorRoot.append(
          new MechanismLigament2d(
              "Rotor",
              1,      // length
              0,       // starting angle
              20,       // line thickness
              new Color8Bit(Color.kPurple)));

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("DyeRotor Motor", dyeRotorCurrentTarget);
    rotorAngleDeg += dyeRotorCurrentTarget * 5; // tune speed
    rotorAngleDeg %= 360; // tune speed
    rotorArm.setAngle(rotorAngleDeg);

  }
}
