package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.DriveConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class Simulation {
  private static Simulation instance;

  private SwerveDriveSimulation swerveDriveSimulation;
  private IntakeSimulation intakeSimulation;

  private StructArrayPublisher<Pose3d> successfulShotsTrajectory =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Simulation/Successful Shots Trajectory", Pose3d.struct)
          .publish();

  private StructArrayPublisher<Pose3d> missedShotsTrajectory =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Simulation/Missed Shots Trajectory", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> publisherSimFuelPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Simulation/Fuel", Pose3d.struct)
          .publish();

  IntegerPublisher publisherSimFuelCount =
      NetworkTableInstance.getDefault().getIntegerTopic("Simulation/Fuel Count").publish();

  private Simulation() {
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

    DriveTrainSimulationConfig driveTrainSimulationConfig =
        DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                COTS.ofMAXSwerve(
                    DCMotor.getNeoVortex(1), DCMotor.getNeo550(1), COTS.WHEELS.COLSONS.cof, 1))
            .withTrackLengthTrackWidth(
                Meters.of(DriveConstants.kWheelBase), Meters.of(DriveConstants.kTrackWidth));

    swerveDriveSimulation =
        new SwerveDriveSimulation(driveTrainSimulationConfig, new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", swerveDriveSimulation, Inches.of(20), Inches.of(6), IntakeSide.FRONT, 36);
  }

  public void startIntake() {
    intakeSimulation.startIntake();
  }

  public void stopIntake() {
    intakeSimulation.stopIntake();
  }

  public int getFuelCount() {
    return intakeSimulation.getGamePiecesAmount();
  }

  public void shootFuel(
      Rotation2d turretAngle, LinearVelocity initialVelocity, Angle shootingAngle) {
    if (getFuelCount() == 0) return;

    intakeSimulation.obtainGamePieceFromIntake();

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                    swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    new Translation2d(), // TODO: create constant
                    swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    turretAngle,
                    Meters.of(0.4), // TODO: create constant
                    initialVelocity,
                    shootingAngle)
                .withProjectileTrajectoryDisplayCallBack(
                    (poses) -> successfulShotsTrajectory.set(poses.toArray(Pose3d[]::new)),
                    (poses) -> missedShotsTrajectory.set(poses.toArray(Pose3d[]::new))));
  }

  public SwerveDriveSimulation getSwerveDriveSimulation() {
    return swerveDriveSimulation;
  }

  public IntakeSimulation getIntakeSimulation() {
    return intakeSimulation;
  }

  public void publish() {
    publisherSimFuelPoses.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    publisherSimFuelCount.set(intakeSimulation.getGamePiecesAmount());
  }

  public static Simulation getInstance() {
    if (instance == null) {
      instance = new Simulation();
    }
    return instance;
  }
}
