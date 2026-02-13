// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  // creates new intake pivot motor
  private SparkFlex pivotMotor =
      new SparkFlex(
          Constants.IntakeConstants.PivotConstants.kIntakePivotCanId, MotorType.kBrushless);

  private SparkClosedLoopController intakePivotController = pivotMotor.getClosedLoopController();
  private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  // creates new roller motor
  private SparkFlex rollerMotor =
      new SparkFlex(
          Constants.IntakeConstants.RollerConstants.kIntakeRollerCanId, MotorType.kBrushless);

  // Configs for Intake - NEEDS TUNING
  public Intake() {
    // Configs for pivotMotor

    pivotMotor.configure(
        Configs.Intake.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Configs for rollerMotor
    rollerMotor.configure(
        Configs.Intake.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    SmartDashboard.putData("Mech2d", intakeMech);
    SmartDashboard.putData("Intake Stow", this.stow());
    SmartDashboard.putData("Intake In", this.intake());
    SmartDashboard.putData("Intake Out", this.extake());
  }

  private void pivotExtend() {
    intakePivotController.setSetpoint(
        Constants.IntakeConstants.PivotConstants.kPivotExtend,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  private void pivotStow() {
    intakePivotController.setSetpoint(
        Constants.IntakeConstants.PivotConstants.kPivotStow,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  private double currentRollerPower = 0;

  private void setRollerPower(double power) {
    rollerMotor.set(power);
    currentRollerPower = power;
  }

  public boolean atSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    }
    return Math.abs(intakePivotController.getSetpoint() - pivotEncoder.getPosition())
        <= IntakeConstants.PivotConstants.kPivotThreshold;
  }

  public Command intake() {
    return this.run(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kIntakeRollerPower);
          pivotExtend();
        });
  }

  public Command extake() {
    return this.run(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kExtakeRollerPower);
          pivotExtend();
        });
  }

  public Command stow() {
    return this.run(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kRollerStop);
          pivotStow();
        });
  }

  Mechanism2d intakeMech = new Mechanism2d(3, 3);
  MechanismRoot2d intakeRoot = intakeMech.getRoot("Intake", 1, 1.5);

  MechanismLigament2d intakeRoller =
      intakeRoot.append(
          new MechanismLigament2d("Intake Roller", 1, 0, 6, new Color8Bit(Color.kAzure)));
  // Two spokes to make it look like a spinning wheel
  MechanismLigament2d rollerSpoke1 =
      intakeRoller.append(
          new MechanismLigament2d("rollerSpoke1", 0.1, -180, 6, new Color8Bit(Color.kYellow)));
  MechanismLigament2d rollerSpoke2 =
      rollerSpoke1.append(
          new MechanismLigament2d("rollerSpoke2", 0.1, 180, 6, new Color8Bit(Color.kBlue)));
  private double rollerSimAngle = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intakeLigament.setAngle(intakePivotAbsoluteEncoder.getPosition());P
    // SmartDashboard.putData("Intake/IntakeMechanism", intakeMech);
    SmartDashboard.putNumber("Intake/Pivot/Current Position", pivotEncoder.getPosition());
    SmartDashboard.putBoolean("Intake/Pivot/At Setpoint?", atSetpoint());

    rollerSimAngle += currentRollerPower * 10;

    rollerSpoke1.setAngle(rollerSimAngle);
    rollerSpoke2.setAngle(rollerSimAngle + 180); // always opposite
  }
}
