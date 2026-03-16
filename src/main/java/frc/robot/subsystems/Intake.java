// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.AgitationConstants;
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
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  // creates new roller motor
  private SparkFlex rollerMotor =
      new SparkFlex(
          Constants.IntakeConstants.RollerConstants.kIntakeRollerCanId, MotorType.kBrushless);
  private RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  private double pivotSetpoint = 0;
  private Pose3d intakePose3d = new Pose3d();

  private Debouncer bottomHardStopDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private Debouncer topHardStopDebouncer = new Debouncer(0.5, DebounceType.kRising);

  // Simulation
  DCMotor pivotMotorSim = DCMotor.getNeoVortex(1);
  DCMotor rollerMotorSim = DCMotor.getNeoVortex(1);
  SparkFlexSim pivotSparkSim = new SparkFlexSim(pivotMotor, pivotMotorSim);
  SparkFlexSim rollerSparkSim = new SparkFlexSim(rollerMotor, rollerMotorSim);
  SparkRelativeEncoderSim pivotEncoderSim = pivotSparkSim.getRelativeEncoderSim();

  SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          pivotMotorSim,
          75, // gearing
          SingleJointedArmSim.estimateMOI(0.15, 4.5), // moment of inertia
          0.15, // length of arm
          Units.degreesToRadians(0), // min angle
          Units.degreesToRadians(Constants.IntakeConstants.PivotConstants.kPivotStow),
          true,
          Units.degreesToRadians(90)); // max angle
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

    // Assume intake is stowed on startup
    pivotEncoder.setPosition(Constants.IntakeConstants.PivotConstants.kPivotStow);
  }

  public void configureBindings() {
    new Trigger(
            () ->
                bottomHardStopDebouncer.calculate(
                    pivotMotor.getAppliedOutput() < 0 && !isPivotMoving()))
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivotMotor.stopMotor();
                  pivotEncoder.setPosition(Constants.IntakeConstants.PivotConstants.kPivotExtend);
                }));

    new Trigger(
            () ->
                topHardStopDebouncer.calculate(
                    pivotMotor.getAppliedOutput() > 0 && !isPivotMoving()))
        .onTrue(
            Commands.runOnce(
                () -> {
                  pivotMotor.stopMotor();
                  pivotEncoder.setPosition(Constants.IntakeConstants.PivotConstants.kPivotStow);
                }));
  }

  private boolean isPivotMoving() {
    return Math.abs(pivotEncoder.getVelocity())
        > IntakeConstants.PivotConstants.kPivotVelocityThreshold;
  }

  private void pivotExtend() {
    pivotMotor.set(IntakeConstants.PivotConstants.kPivotDownPower);
  }

  private void pivotStow() {
    pivotMotor.set(IntakeConstants.PivotConstants.kPivotUpPower);
  }

  private void setRollerPower(double power) {
    rollerMotor.set(power);
  }

  public Pose3d getIntakePose3d() {
    return intakePose3d;
  }

  public double getIntakePivotPosition() {
    return pivotEncoder.getPosition();
  }

  /** True when the pivot has reached its target position */
  public boolean atSetpoint() {
    return Math.abs(intakePivotController.getSetpoint() - pivotEncoder.getPosition())
        <= IntakeConstants.PivotConstants.kPivotThreshold;
  }

  // Intake Simulation - Mech2d
  Mechanism2d intakeMech = new Mechanism2d(1, 1);
  MechanismRoot2d intakeRoot = intakeMech.getRoot("Intake", 0.85, 0.1);

  MechanismLigament2d intakeBar =
      intakeRoot.append(
          new MechanismLigament2d("Intake Roller", 0.25, 90, 3, new Color8Bit(Color.kBlue)));

  MechanismLigament2d intakeRollerMotorSim =
      intakeBar.append(
          new MechanismLigament2d("Roller Motor", 0.1, 180, 3, new Color8Bit(Color.kWhite)));

  /** Extends pivot and runs rollers inward. Stops rollers on end. */
  public Command intake() {
    return Commands.runEnd(
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
    return Commands.runEnd(
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
    return this.runOnce(
        () -> {
          setRollerPower(Constants.IntakeConstants.RollerConstants.kRollerStop);
          pivotStow();
        });
  }

  public Command extend() {
    return this.runOnce(
        () -> {
          pivotExtend();
        });
  }

  /**
   * Cycles the intake stow/extend with rollers to agitate fuel. Runs for a fixed number of cycles
   * and then finishes, leaving the intake extended with rollers stopped.
   */
  public Command agitate() {
    Command sequence = Commands.none();
    for (int i = 0; i < AgitationConstants.kAgitationCount; i++) {
      sequence =
          sequence.andThen(
              // Stow phase: retract pivot, stop roller
              this.run(
                      () -> {
                        setRollerPower(IntakeConstants.RollerConstants.kRollerStop);
                        pivotStow();
                      })
                  .withTimeout(AgitationConstants.kStowDurationSeconds),
              // Extend phase: extend pivot, run roller
              this.run(
                      () -> {
                        setRollerPower(IntakeConstants.RollerConstants.kIntakeRollerPower);
                        pivotExtend();
                      })
                  .withTimeout(AgitationConstants.kExtendDurationSeconds));
    }
    return sequence.finallyDo(() -> setRollerPower(IntakeConstants.RollerConstants.kRollerStop));
  }

  public Command agitateOut() {
    return this.runOnce(
        () -> {
          pivotMotor.set(-.2);
        });
  }

  public Command agitateIn() {
    return this.runOnce(
        () -> {
          pivotMotor.set(.2);
        });
  }

  @Override
  public void periodic() {
    intakeBar.setAngle(pivotEncoder.getPosition());
    intakeRollerMotorSim.setAngle(Units.rotationsToDegrees(rollerEncoder.getPosition()));

    SmartDashboard.putNumber("Intake/Pivot/Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Intake/Pivot/Velocity", pivotEncoder.getVelocity());
    SmartDashboard.putNumber("Intake/Pivot/Setpoint", pivotSetpoint);
    SmartDashboard.putBoolean("Intake/Pivot/At Setpoint?", atSetpoint());

    // 3d SIM
    intakePose3d =
        new Pose3d(
            0.35,
            0,
            0.232,
            new Rotation3d(
                0.0,
                Units.degreesToRadians(
                    IntakeConstants.PivotConstants.kPivotStow - getIntakePivotPosition()),
                0.0));
  }

  @Override
  public void simulationPeriodic() {
    pivotSim.setInputVoltage(
        pivotSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    pivotSim.update(0.02);

    SmartDashboard.putNumber(
        "Intake/Pivot/Sim Position", Units.radiansToDegrees(pivotSim.getAngleRads()));

    pivotSparkSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec() * 75),
        RobotController.getBatteryVoltage(),
        0.02);

    rollerSim.setInputVoltage(
        rollerSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    rollerSim.update(0.02);

    rollerSparkSim.iterate(
        rollerSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
  }
}
