// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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

public class DyeRotor extends SubsystemBase {

  private SparkFlex dyeRotorMotor =
      new SparkFlex(Constants.DyeRotorConstants.kDyeRotorMotorCanID, MotorType.kBrushless);
  private double dyeRotorCurrentTarget = 0;
  private double rotorAngleDeg = 0;
  private Pose3d pose = new Pose3d();

  /** Creates a new Dyerotor. */
  public DyeRotor() {
    dyeRotorMotor.configure(
        Configs.DyeRotor.dyeRotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rotorArm.setAngle(45);
    SmartDashboard.putData("Dye Rotor Mech", mech2d);
  }

  public Command start() {
    return this.run(
        () -> {
          dyeRotorCurrentTarget = Constants.DyeRotorConstants.kDyeRotorPower;
          dyeRotorMotor.set(Constants.DyeRotorConstants.kDyeRotorPower);
          double spinRateDegPerSec = 90.0; // change to taste
          double spinRateRadPerSec = Math.toRadians(spinRateDegPerSec);
          double spinAngle = (Timer.getFPGATimestamp() * spinRateRadPerSec) % (2.0 * Math.PI);
        });
  }
  ;

  public Command stop() {
    return this.run(
        () -> {
          dyeRotorCurrentTarget = 0;
          dyeRotorMotor.set(0);

          Pose3d[] finalRobotPose =
              new Pose3d[] {new Pose3d(0, 0, 0.02, new Rotation3d(0.0, 0.0, 0.0))};
        });
  }

  public Pose2d getPose() {
    // Return a default pose at the origin; replace with a real pose provider if available.
    return new Pose2d();
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

  public Pose3d getDyePose() {
    return pose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("DyeRotor Motor", dyeRotorCurrentTarget);
    rotorAngleDeg += dyeRotorCurrentTarget * 5; // tune speed
    rotorAngleDeg %= 360; // tune speed
    rotorArm.setAngle(rotorAngleDeg);

    Pose2d currentPose = getPose();
    pose =
        new Pose3d(
            0,
            0,
            0.02,
            new Rotation3d(
                0.0,
                0.0,
                currentPose.getRotation().getRadians() + Units.degreesToRadians(rotorAngleDeg)));
  }
  ;
}
