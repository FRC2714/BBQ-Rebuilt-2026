package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DyeRotor;
import frc.robot.subsystems.Shooter;

public class StateMachine extends SubsystemBase {
  private final DriveSubsystem m_drivetrain;
  private final Shooter m_shooter;
  private final Publisher m_publisher;
  private final Intake m_intake;
  private final DyeRotor m_dyeRotor;

  private static State m_state = State.Idle;

  private static boolean isNotClimbing() {
    return !(m_state == State.Climbing);
  }

  enum State {
    Idle,
    Shooting,
    Climbing
  }

  public StateMachine(DriveSubsystem drivetrain, Shooter shooter, Intake intake) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_intake = intake;
    m_dyeRotor = dyeRotor;

    m_publisher = new Publisher(m_drivetrain, m_shooter, m_intake);
  }

  StructArrayPublisher<Pose3d> publisherZeroedComponentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct)
          .publish();

  StructArrayPublisher<Pose3d> publisherFinalComponentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("FinalComponentPoses", Pose3d.struct)
          .publish();

  public State getState() {
    return m_state;
  }

  public void setState(State state) {
    m_state = state;
  }

  // Generalization of updating the targets
  private static final double kLatencyCompensation = 0.1;
  private static final double kBaselineHorizontalVelocity = 6.0;

  private void aimAt(Translation2d target, Translation2d robotPosition, Rotation2d robotHeading) {
    Translation2d robotVelocity = m_drivetrain.getFieldRelativeVelocity();

    Translation2d futurePosition = robotPosition.plus(robotVelocity.times(kLatencyCompensation));

    Translation2d toGoal = target.minus(futurePosition);
    Translation2d targetDirection = toGoal.div(toGoal.getNorm());
    Translation2d targetVelocity = targetDirection.times(kBaselineHorizontalVelocity);
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    double distanceToTarget = toGoal.getNorm();

    Translation2d virtualTarget =
        futurePosition.plus(shotVelocity.div(shotVelocity.getNorm()).times(distanceToTarget));

    Translation2d robotToTarget = virtualTarget.minus(robotPosition);
    Rotation2d fieldAngle = robotToTarget.getAngle();
    Rotation2d turretAngle = fieldAngle.minus(robotHeading);

    m_shooter.updateTurretTarget(turretAngle.getDegrees());
    m_shooter.updateHoodTarget(distanceToTarget);
    m_shooter.updateFlywheelTarget(distanceToTarget);
  }

  // intake commands
  public Command intakeSequence() {
    return (m_intake.intake().onlyIf(StateMachine::isNotClimbing));
  }

  public Command extakeSequence() {
    return (m_intake.extake().onlyIf(StateMachine::isNotClimbing));
  }

  public Command stowSequence() {
    return (m_intake.stow().onlyIf(StateMachine::isNotClimbing));
  }

  @Override
  public void periodic() {
    Translation2d robotPosition = m_drivetrain.getPose().getTranslation();
    Rotation2d robotHeading = m_drivetrain.getPose().getRotation();

    if (m_drivetrain.isInAllianceZone()) {
      aimAt(Field.getAllianceHub().toTranslation2d(), robotPosition, robotHeading);
      m_publisher.publish();
      return;
    }

    double airstrikeX = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/x", 0));
    double airstrikeY = Units.inchesToMeters(SmartDashboard.getNumber("airstrike/y", 0));

    if (airstrikeX == 0 && airstrikeY == 0) {
      m_shooter.updateTurretTarget(0.0);
      m_publisher.publish();
      return;
    }

    Translation2d airstrikeTarget = new Translation2d(airstrikeX, airstrikeY);
    aimAt(airstrikeTarget, robotPosition, robotHeading);
    m_publisher.publish();

    Pose3d[] zeroRobotPose = new Pose3d[1];
    for (int i = 0; i < zeroRobotPose.length; i++) {
      zeroRobotPose[i] = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    }
    publisherZeroedComponentPoses.set(zeroRobotPose);
    Pose3d[] finalRobotPose = new Pose3d[] {m_dyeRotor.getDyePose()};
    publisherFinalComponentPoses.set(finalRobotPose);
  }
}
;
