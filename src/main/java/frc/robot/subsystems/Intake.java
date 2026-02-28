// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import frc.robot.Simulation;

/** Controls the intake pivot and roller. Extends to collect fuel, stows to retract. */
public class Intake extends SubsystemBase {

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
  private RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  private double pivotSetpoint = 0;

  // Simulation
  DCMotor pivotMotorSim = DCMotor.getNeoVortex(1);
  DCMotor rollerMotorSim = DCMotor.getNeoVortex(1);
  SparkFlexSim pivotSparkSim = new SparkFlexSim(pivotMotor, pivotMotorSim);
  SparkFlexSim rollerSparkSim = new SparkFlexSim(rollerMotor, rollerMotorSim);
  SparkAbsoluteEncoderSim pivotEncoderSim = pivotSparkSim.getAbsoluteEncoderSim();

  SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          pivotMotorSim,
          40, // gearing
          SingleJointedArmSim.estimateMOI(0.15, 4.5), // moment of inertia
          0.15, // length of arm
          Units.degreesToRadians(0), // min angle
          Units.degreesToRadians(Constants.IntakeConstants.PivotConstants.kPivotStow + 20),
          true,
          0); // max angle
  FlywheelSim rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(rollerMotorSim, 0.0005, 1), rollerMotorSim);

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
    SmartDashboard.putData("Intake/Mech2d", intakeMech);
  }

  private void pivotExtend() {
    pivotSetpoint = Constants.IntakeConstants.PivotConstants.kPivotExtend;
    intakePivotController.setSetpoint(pivotSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void pivotStow() {
    pivotSetpoint = Constants.IntakeConstants.PivotConstants.kPivotStow;
    intakePivotController.setSetpoint(pivotSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void setRollerPower(double power) {
    rollerMotor.set(power);
  }

  /** True when the pivot has reached its target position. Always true in sim. */
  public boolean atSetpoint() {
    if (Robot.isSimulation()) {
      return true;
    }
    return Math.abs(intakePivotController.getSetpoint() - pivotEncoder.getPosition())
        <= IntakeConstants.PivotConstants.kPivotThreshold;
  }

  // Intake Simulation - Mech2d
  Mechanism2d intakeMech = new Mechanism2d(1, 1);
  MechanismRoot2d intakeRoot = intakeMech.getRoot("Intake", 0.15, 0.1);

  MechanismLigament2d intakeBar =
      intakeRoot.append(
          new MechanismLigament2d("Intake Roller", 0.25, 90, 3, new Color8Bit(Color.kBlue)));

  MechanismLigament2d intakeRollerMotorSim =
      intakeBar.append(
          new MechanismLigament2d("Roller Motor", 0.1, 180, 3, new Color8Bit(Color.kWhite)));

  /** Extends pivot and runs rollers inward. Stops rollers on end. */
  public Command intake() {
    return this.runEnd(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kIntakeRollerPower);
          pivotExtend();

          if (Robot.isSimulation()) Simulation.getInstance().startIntake();
        },
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kRollerStop);

          if (Robot.isSimulation()) Simulation.getInstance().stopIntake();
        });
  }

  /** Extends pivot and runs rollers outward (eject). Stops rollers on end. */
  public Command extake() {
    return this.runEnd(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kExtakeRollerPower);
          pivotExtend();
        },
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kRollerStop);
        });
  }

  /** Retracts pivot and stops rollers. */
  public Command stow() {
    return this.run(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kRollerStop);
          pivotStow();
        });
  }

  @Override
  public void periodic() {
    intakeBar.setAngle(pivotEncoder.getPosition());
    intakeRollerMotorSim.setAngle(Units.rotationsToDegrees(rollerEncoder.getPosition()));

    SmartDashboard.putNumber("Intake/Pivot/Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSetpoint);
    SmartDashboard.putBoolean("Intake/Pivot/At Setpoint?", atSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    pivotSim.setInputVoltage(
        pivotSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    pivotSim.update(0.02);

    pivotEncoderSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec()), 0.02);

    // TODO(jan): I think this should have the gear ratio multiplied, but when it is, the pid goes
    // crazy
    pivotSparkSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec()),
        RobotController.getBatteryVoltage(),
        0.02);

    rollerSim.setInputVoltage(
        rollerSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    rollerSim.update(0.02);

    rollerSparkSim.iterate(
        rollerSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
  }
}
