package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FlywheelSetpoints;
import frc.robot.Constants.ShooterConstants.HoodSetpoints;
import frc.robot.Constants.ShooterConstants.TurretSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.utils.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
  // turret motors and controllers
  private SparkFlex turretMotor =
      new SparkFlex(Constants.ShooterConstants.kTurretCanId, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();

  private RelativeEncoder turretRelativeEncoder = turretMotor.getExternalEncoder();

  private SparkFlex hoodMotor =
      new SparkFlex(Constants.ShooterConstants.kHoodCanId, MotorType.kBrushless);
  private SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
  private RelativeEncoder hoodRelativeEncoder = hoodMotor.getExternalEncoder();

  private SparkFlex flywheelMotorLeader =
      new SparkFlex(Constants.ShooterConstants.kFlywheelLeaderMotorId, MotorType.kBrushless);
  private SparkClosedLoopController flywheelController =
      flywheelMotorLeader.getClosedLoopController();
  private RelativeEncoder flywheelRelativeEncoder = flywheelMotorLeader.getExternalEncoder();

  private SparkFlex flywheelMotorFollower =
      new SparkFlex(Constants.ShooterConstants.kFlywheelFollowerMotorId, MotorType.kBrushless);

  // limit switches
  private SparkLimitSwitch turretForwardLimitSwitch = turretMotor.getForwardLimitSwitch();
  private SparkLimitSwitch turretReverseLimitSwitch = turretMotor.getReverseLimitSwitch();

  private double turretCurrentTarget = TurretSetpoints.kStow;

  private double hoodTarget = HoodSetpoints.kStow;

  private double flywheelTargetSpeed = FlywheelSetpoints.kStow;

  private InterpolatingTreeMap hoodAngleMap;
  private InterpolatingTreeMap flywheelSpeedMap;
  private DriveSubsystem m_DriveSubsystem;

    // Simulation setup and variables
  private DCMotor armMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim m_hoodPivotSim =
      new SingleJointedArmSim(
          armMotorModel,
          Constants.SimulationRobotConstants.kHoodReduction,
          SingleJointedArmSim.estimateMOI(
          Constants.SimulationRobotConstants.kHoodLength, Constants.SimulationRobotConstants.kHoodMass),
          Constants.SimulationRobotConstants.kHoodLength,
          Constants.SimulationRobotConstants.kHoodMinAngleRads,
          Constants.SimulationRobotConstants.kHoodMaxAngleRads,
          true,
          SimulationRobotConstants.kHoodMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsytem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Coral Intake Root", 25.1, 0);
  private final MechanismLigament2d hoodPivotMechanism =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Hood Pivot",
              SimulationRobotConstants.kHoodLength,
              Constants.ShooterConstants.HoodSetpoints.kZeroOffsetDegrees));

  // Creates a hood
  public Shooter() {
    turretMotor.configure(
        Configs.Shooter.turretConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodMotor.configure(
        Configs.Shooter.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelMotorLeader.configure(
        Configs.Shooter.flywheelConfigLeader,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    flywheelMotorFollower.configure(
        Configs.Shooter.flywheelConfigFollower,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodAngleMap = new InterpolatingTreeMap();
    flywheelSpeedMap = new InterpolatingTreeMap();

    populateHoodAngleMap();
    populateFlywheelSpeedMap();

    SmartDashboard.putData("Mech2D's/Hood Pivot", m_mech2d);

    armMotorSim = new SparkFlexSim(hoodMotor, armMotorModel);

  }

  public void populateHoodAngleMap() {
    // Populate hood angle with data
  }

  public void populateFlywheelSpeedMap() {
    // Populate flywheel speed with data
  }

  // Get positions
  public double getTurretPosition() {
    return turretRelativeEncoder.getPosition();
  }

  public double getHoodPosition() {
    return hoodRelativeEncoder.getPosition();
  }

  // Update the target positions
  public void updateTurretTarget(double updateValue) {
    turretCurrentTarget =
        MathUtil.clamp(
            updateValue,
            Constants.ShooterConstants.kTurretMinRange,
            Constants.ShooterConstants.kTurretMaxRange);
  }

  // public double updateHoodTarget() {
  //   return hoodAngleMap.getInterpolated(m_DriveSubsystem.getDistanceToHub());
  // }

  // public double updateFlyWheelSpeedTarget() {
  //   return flywheelSpeedMap.getInterpolated(m_DriveSubsystem.getDistanceToHub());
  // }

  public void setFlywheelSpeed(double speed) {
    flywheelTargetSpeed = speed;
  }

  public double getFlywheelSpeed() {
    return flywheelController.getSetpoint();
  }

  public void setHoodAngle(double angle) {
    hoodTarget = angle;
  }

  public double getHoodAngle() {
    return hoodController.getSetpoint();
  }

  public Command startShooter() {
    return this.run(
        () -> {
          setFlywheelSpeed(Constants.ShooterConstants.FlywheelSetpoints.kStartSpeed);
        });
  }

  public Command stopShooter() {
    return this.run(
        () -> {
          setFlywheelSpeed(0);
        });
  }

  public Command stowShooter() {
    return this.run(
        () -> {
          setHoodAngle(Constants.ShooterConstants.HoodSetpoints.kStow);
        });
  }

  // Runs every 20ms
  @Override
  public void periodic() {
    turretController.setSetpoint(turretCurrentTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // hoodController.setSetpoint(updateHoodTarget(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // flywheelController.setSetpoint(
    //     updateFlyWheelSpeedTarget(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("Hood Angle", hoodRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Flywheel Speed", flywheelRelativeEncoder.getVelocity());
  }

   @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_hoodPivotSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_hoodPivotSim.update(0.020);

    // Iterate the arm SPARK simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_hoodPivotSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java

     SmartDashboard.putNumber("Hood Angle", hoodRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Flywheel Speed", flywheelRelativeEncoder.getVelocity());
  }
}
