// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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

/** Indexes fuel through the robot using a spinning rotor. Supports pause/resume during shooting. */
public class DyeRotor extends SubsystemBase {

  private SparkFlex dyeRotorMotor =
      new SparkFlex(Constants.DyeRotorConstants.kDyeRotorMotorCanID, MotorType.kBrushless);
  private RelativeEncoder encoder = dyeRotorMotor.getEncoder();
  private double dyeRotorCurrentTarget = 0;
  private boolean paused = false;

  private Pose3d pose = new Pose3d();

  // Simulation
  DCMotor motor = DCMotor.getNeoVortex(1);
  private SparkFlexSim dyeRotorSim = new SparkFlexSim(dyeRotorMotor, motor);
  private static final double MOMENT_OF_INERTIA = 0.00032; // kg*m^2
  private static final double GEARING = 56.25; // 1:1 if direct drive
  private FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(motor, MOMENT_OF_INERTIA, GEARING), motor);

  public DyeRotor() {
    dyeRotorMotor.configure(
        Configs.DyeRotor.dyeRotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rotorArm.setAngle(45);
    SmartDashboard.putData("Dye Rotor/Mech2d", mech2d);
  }

  /** Runs the rotor (respects pause state). Clears pause on start. */
  public Command start() {
    return this.run(
            () -> {
              dyeRotorCurrentTarget = paused ? 0 : Constants.DyeRotorConstants.kDyeRotorPower;
              dyeRotorMotor.set(dyeRotorCurrentTarget);
            })
        .beforeStarting(
            () -> {
              paused = false;
            });
  }

  /** Stops the rotor immediately. */
  public Command stop() {
    return this.runOnce(
        () -> {
          dyeRotorCurrentTarget = 0;
          dyeRotorMotor.set(0);
        });
  }

  /** Momentarily run the rotor inreverse. */
  public Command unjam() {
    return this.run(
            () -> {
              dyeRotorCurrentTarget = paused ? 0 : -Constants.DyeRotorConstants.kDyeRotorPower;
              dyeRotorMotor.set(dyeRotorCurrentTarget);
            })
        .withTimeout(0.15)
        .andThen(stop());
  }

  /** Starts the rotor directly (no command). Used by auto event markers. */
  public void startDirect() {
    paused = false;
    dyeRotorCurrentTarget = Constants.DyeRotorConstants.kDyeRotorPower;
    dyeRotorMotor.set(dyeRotorCurrentTarget);
  }

  /** Stops the rotor directly (no command). Used by auto event markers. */
  public void stopDirect() {
    dyeRotorCurrentTarget = 0;
    dyeRotorMotor.set(0);
  }

  /** Pauses the rotor. The {@link #start()} command will output 0 until resumed. */
  public void pause() {
    paused = true;
    dyeRotorCurrentTarget = 0;
    dyeRotorMotor.set(0);
  }

  /** Resumes the rotor after a pause. */
  public void resume() {
    paused = false;
    dyeRotorCurrentTarget = Constants.DyeRotorConstants.kDyeRotorPower;
    dyeRotorMotor.set(dyeRotorCurrentTarget);
  }

  // Mech2d for DyeRotor
  private final Mechanism2d mech2d = new Mechanism2d(60, 60); // width, height in "virtual units"

  private final MechanismRoot2d rotorRoot = mech2d.getRoot("DyeRotorRoot", 29.83, 0);

  private final MechanismLigament2d rotorArm =
      rotorRoot.append(
          new MechanismLigament2d(
              "Rotor",
              1, // length
              0, // starting angle
              20, // line thickness
              new Color8Bit(Color.kPurple)));

  public Pose3d getPose3d() {
    return pose;
  }

  /** Returns rotor position in output rotations (after gearing). */
  public double getRotorPosition() {
    return encoder.getPosition() / GEARING;
  }

  /** True if the rotor has a non-zero target. */
  public boolean isRunning() {
    return dyeRotorCurrentTarget != 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Dye Rotor/Setpoint", dyeRotorCurrentTarget);
    rotorArm.setAngle(Units.rotationsToDegrees(encoder.getPosition()));

    pose =
        new Pose3d(
            0.058,
            0,
            0.2,
            new Rotation3d(0.0, 0.0, Units.rotationsToRadians(-encoder.getPosition() / GEARING)));
  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInputVoltage(
        dyeRotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
    flywheelSim.update(0.02);

    dyeRotorSim.iterate(
        flywheelSim.getAngularVelocityRPM() * GEARING, RobotController.getBatteryVoltage(), 0.02);
  }
}
