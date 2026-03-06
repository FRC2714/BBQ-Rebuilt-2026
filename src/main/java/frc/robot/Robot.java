// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utils.LimelightHelpers;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CanandEventLoop.getInstance();

    LimelightHelpers.Flush();

    if (!Robot.isSimulation()) {
      DataLogManager.start();
      var log = DataLogManager.getLog();
      log.addSchema(Pose2d.proto);
      log.addSchema(ChassisSpeeds.proto);
      log.addSchema(Pose2d.struct);
      log.addSchema(Pose3d.struct);

      URCL.start();

      StatusLogger.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    LimelightHelpers.Flush();
  }

  @Override
  public void disabledPeriodic() {
    double heading = m_robotContainer.m_robotDrive.getHeading();
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.kFrontRightName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kFrontLeftName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kRearLeftName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kRearRightName, heading, 0, 0, 0, 0, 0);

    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontRightName, 1);
    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontLeftName, 1);
    LimelightHelpers.SetIMUMode(LimelightConstants.kRearLeftName, 1);
    LimelightHelpers.SetIMUMode(LimelightConstants.kRearRightName, 1);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().resetFieldForAuto();
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontRightName, 4); // 4 is internal imu + gyro
    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontLeftName, 4);
    LimelightHelpers.SetIMUMode(LimelightConstants.kRearLeftName, 4);
    LimelightHelpers.SetIMUMode(LimelightConstants.kRearRightName, 4);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double heading = m_robotContainer.m_robotDrive.getHeading();
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.kFrontRightName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kFrontLeftName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kRearLeftName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kRearRightName, heading, 0, 0, 0, 0, 0);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().schedule(m_robotContainer.m_stateMachine.unclimb());

    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontRightName, 4); // 4 is internal imu + gyro
    LimelightHelpers.SetIMUMode(LimelightConstants.kFrontLeftName, 4);
    LimelightHelpers.SetIMUMode(LimelightConstants.kRearLeftName, 4);
    LimelightHelpers.SetIMUMode(LimelightConstants.kRearRightName, 4);
    // LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.kFrontRightName, .005);
    // LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.kFrontLeftName, .005);
    // LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.kRearLeftName, .005);
    // LimelightHelpers.SetIMUAssistAlpha(LimelightConstants.kRearRightName, .005);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double heading = m_robotContainer.m_robotDrive.getHeading();
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.kFrontRightName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kFrontLeftName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kRearLeftName, heading, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(LimelightConstants.kRearRightName, heading, 0, 0, 0, 0, 0);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().placeGamePiecesOnField();
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
  }
}
