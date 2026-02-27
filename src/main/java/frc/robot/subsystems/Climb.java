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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Robot;

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

  public Command deploy() {
    return this.runEnd(
        () -> {
          leftMotor.set(Constants.ClimbConstants.kExtendSpeed);
          rightMotor.set(Constants.ClimbConstants.kExtendSpeed);
          moveToDeploy();
        },
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        });
  }

  public Command climb() {
    return this.runEnd(
        () -> {
          leftMotor.set(Constants.ClimbConstants.kExtendSpeed);
          rightMotor.set(Constants.ClimbConstants.kExtendSpeed);
          moveToClimb();
        },
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        });
  }

  public Command unclimb() {
    return this.runEnd(
        () -> {
          leftMotor.set(Constants.ClimbConstants.kRetractSpeed);
          rightMotor.set(Constants.ClimbConstants.kRetractSpeed);
          moveToRetract();
        },
        () -> {
          leftMotor.set(0);
          rightMotor.set(0);
        });
  }

  private void moveToDeploy() {
    setpoint = Constants.ClimbConstants.kExtendSetpoint;
    leftController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void moveToClimb() {
    setpoint = Constants.ClimbConstants.kClimbSetpoint;
    leftController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void moveToRetract() {
    setpoint = Constants.ClimbConstants.kRetractSetpoint;
    leftController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean atSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    }
    return Math.abs(leftController.getSetpoint() - leftMotor.getEncoder().getPosition())
            <= Constants.ClimbConstants.kPositionTolerance
        && Math.abs(rightController.getSetpoint() - rightMotor.getEncoder().getPosition())
            <= Constants.ClimbConstants.kPositionTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb/At Setpoint?", atSetpoint());
  }
}
