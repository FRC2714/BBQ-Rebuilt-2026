// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the left and right climb motors. Supports deploy, climb, and retract positions. */
public class Climb extends SubsystemBase {
  // private final SparkFlex leftMotor =
  //     new SparkFlex(Constants.ClimbConstants.kClimbMotorCanID, MotorType.kBrushless);
  // private final AbsoluteEncoder absEnc = leftMotor.getAbsoluteEncoder();
  // private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();

  private Pose3d climbPose = new Pose3d();

  DCMotor climbMotorSim = DCMotor.getNeoVortex(2);
  // SparkFlexSim leftMotorSim = new SparkFlexSim(leftMotor, climbMotorSim);
  SingleJointedArmSim climbSim =
      new SingleJointedArmSim(
          climbMotorSim,
          125,
          SingleJointedArmSim.estimateMOI(0.1, 1),
          0.1,
          Units.degreesToRadians(0),
          Units.degreesToRadians(270),
          false,
          0);

  public Climb() {
    // leftMotor.configure(
    //     Configs.Climb.climbConfig, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
  }

  // /** Extends the climber arms to the deploy position. Stops motors on end. */
  // public Command deploy() {
  //   return this.run(
  //       () -> {
  //         leftController.setSetpoint(
  //             ClimbConstants.kDeployedSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  //       });
  // }

  /** Pulls the robot up to the climb position. Stops motors on end. */
  // public Command climb() {
  //   return this.run(
  //       () -> {
  //         // Use slower slot 0 for safety
  //         leftController.setSetpoint(ClimbConstants.kClimbSetpoint, ControlType.kPosition);
  //       });
  // }

  /** Retracts the climber arms back down. Stops motors on end. */
  // public Command unclimb() {
  //   return this.run(
  //       () -> {
  //         leftController.setSetpoint(
  //             ClimbConstants.kStowSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot1);
  //       });
  // }

  // /** True when both motors are within tolerance of the target position */
  // public boolean atSetpoint() {
  //   return Math.abs(leftController.getSetpoint() - absEnc.getPosition())
  //       <= Constants.ClimbConstants.kPositionTolerance;
  // }

  public Pose3d getClimbPose3d() {
    return climbPose;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Climb/At Setpoint?", atSetpoint());
    // SmartDashboard.putNumber("Climb/Position", absEnc.getPosition());

    // climbPose =
    //     new Pose3d(
    //         -0.3225,
    //         0.0,
    //         0.58,
    //         new Rotation3d(0.0, Units.degreesToRadians(absEnc.getPosition()), 0.0));
  }

  @Override
  public void simulationPeriodic() {
    // climbSim.setInput(leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    // climbSim.update(0.02);

    // leftMotorSim.iterate(
    //     Units.radiansPerSecondToRotationsPerMinute(climbSim.getVelocityRadPerSec()),
    //     RobotController.getBatteryVoltage(),
    //     0.02);
  }
}
