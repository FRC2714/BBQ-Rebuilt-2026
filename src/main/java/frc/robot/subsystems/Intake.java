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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private enum PivotSetpoints {
    STOW,
    INTAKE,
    EXTAKE,
    SCORE,
  }

  private enum RollerSetpoints {
    STOW,
    INTAKE,
    EXTAKE,
    STOP,
  }

  // creates new intake pivot motor
  private SparkFlex pivotMotor =
      new SparkFlex(
          Constants.IntakeConstants.PivotConstants.kIntakePivotCanId, MotorType.kBrushless);
  private SparkClosedLoopController intakePivotController = pivotMotor.getClosedLoopController();
  private AbsoluteEncoder intakePivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
  private ArmFeedforward pivotFF =
      new ArmFeedforward(0, Constants.IntakeConstants.PivotConstants.kPivotkG, 0);

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

  private void setRollerPower(double power) {
    rollerMotor.set(power);
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

  // Mech2d
  Mechanism2d intakeMech = new Mechanism2d(2, 1);
  MechanismRoot2d intakeRoot = intakeMech.getRoot("Intake", 1, 0.75);
  MechanismLigament2d intakeLigament1 = new MechanismLigament2d("intakeLigament1", 0.5, -135);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake/Alive", true);
    SmartDashboard.putData("Intake/Mech2d", intakeMech);
  }
}
