// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Shooter m_shooter = new Shooter();
  public final DyeRotor m_dyeRotor = new DyeRotor();
  public final Intake m_intake = new Intake();
  private final Climb m_climb = new Climb();

  Joystick m_operatorBox = new Joystick(1); // operator controller 1
  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final JoystickButton leftSwitch = new JoystickButton(m_operatorBox, 1);
  public final JoystickButton middleSwitch = new JoystickButton(m_operatorBox, 2);
  private final JoystickButton rightSwitch = new JoystickButton(m_operatorBox, 3);
  private final JoystickButton extendIntakeButton = new JoystickButton(m_operatorBox, 4);
  private final JoystickButton retractIntakeButton = new JoystickButton(m_operatorBox, 7);
  private final JoystickButton button5 = new JoystickButton(m_operatorBox, 5);
  private final JoystickButton button9 = new JoystickButton(m_operatorBox, 9);
  private final JoystickButton button8 = new JoystickButton(m_operatorBox, 8);
  private final JoystickButton zeroHoodButton = new JoystickButton(m_operatorBox, 6);

  final StateMachine m_stateMachine =
      new StateMachine(m_robotDrive, m_shooter, m_intake, m_dyeRotor, m_climb, m_driverController);

  private SendableChooser<Command> autoChooser;

  private final Trigger rumble = new Trigger(() -> m_stateMachine.phaseShift());
  private final Trigger warningRumble = new Trigger(() -> m_stateMachine.phaseShiftWarning());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NAMED COMMANDS FOR PATHPLANNER
    NamedCommands.registerCommand("SCORE", m_stateMachine.shoot());
    NamedCommands.registerCommand("AGITATE", m_intake.agitate());

    NamedCommands.registerCommand(
        "ENABLE_PASSING", Commands.runOnce(() -> m_stateMachine.enablePassing()));
    NamedCommands.registerCommand(
        "DISABLE_PASSING", Commands.runOnce(() -> m_stateMachine.disablePassing()));

    NamedCommands.registerCommand("STOP_SHOOTING", m_stateMachine.stopShoot());
    NamedCommands.registerCommand("INTAKE", m_stateMachine.intakeSequence());
    NamedCommands.registerCommand(
        "STOW_INTAKE", m_stateMachine.stowSequenceAuto(AutoConstants.kStowTimeout));
    NamedCommands.registerCommand("PRELOAD", m_stateMachine.preloadCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_stateMachine.configureBindings();
    button5.onTrue(m_stateMachine.zeroPoseAuto());

    m_driverController.back().onTrue(m_stateMachine.zeroPoseAuto());
    m_driverController
        .start()
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroDriverHeading(), m_robotDrive));

    button9.onTrue(m_shooter.zeroTurretSequenceRight());

    button8.onTrue(m_shooter.zeroTurretSequenceLeft());
    leftSwitch.whileTrue(m_stateMachine.toggleOverride());

    // Shooter moved to state machine bindings

    // intake keybinds
    m_driverController.rightTrigger().whileTrue(m_stateMachine.intakeSequence());
    m_driverController
        .leftTrigger()
        .onTrue(m_stateMachine.shoot())
        .onFalse(m_stateMachine.stopShoot());
    m_driverController.b().onTrue(m_stateMachine.stowSequence());
    m_driverController.rightBumper().whileTrue(m_stateMachine.extakeSequence());
    m_driverController.a().onTrue(m_dyeRotor.unjam());

    // m_driverController.povLeft().onTrue(m_stateMachine.deplo\[]yClimber());
    // m_driverController.povUp().onTrue(m_stateMachine.climb());
    // m_driverController.povDown().onTrue(m_stateMachine.unclimb());

    rumble.onTrue(
        new StartEndCommand(
                () -> {
                  m_driverController.setRumble(RumbleType.kLeftRumble, 1.0);
                  m_driverController.setRumble(RumbleType.kRightRumble, 1.0);
                },
                () -> {
                  m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
                  m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
                })
            .withTimeout(3));

    warningRumble.onTrue(
        new StartEndCommand(
                () -> {
                  m_driverController.setRumble(RumbleType.kLeftRumble, 0.5);
                  m_driverController.setRumble(RumbleType.kRightRumble, 0.5);
                },
                () -> {
                  m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
                  m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
                })
            .withTimeout(0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    return autoChooser.getSelected();
  }
}
