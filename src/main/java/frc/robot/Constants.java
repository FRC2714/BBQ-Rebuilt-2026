// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kBumperThickness = Units.inchesToMeters(3.5); // Needs to change

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = false;

    public static final SimpleMotorFeedforward kDriveFeedforward =
        new SimpleMotorFeedforward(.11204, 2.0817, .15569);
    // test 2 (0.2049,1.9347, 0.37092)
    public static final SimpleMotorFeedforward kTurningFeedforward =
        new SimpleMotorFeedforward(0, 0, 0);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Command timeouts in seconds (needs to be tested)
    public static final double kShootInitialTimeout = 1.5;
    public static final double kShootTimeout = 2.25;
    public static final double kIntakeTimeout = 10;
    public static final double kExtakeTimeout = 1.75;
    public static final double kStowTimeout = 0.75;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6780;
  }

  public static final class LimelightConstants {
    public static final String kFrontRightName = "limelight-fr";
    public static final String kFrontLeftName = "limelight-fl";
    public static final String kRearLeftName = "limelight-rl";
    public static final String kRearRightName = "limelight-rr";
    public static final String[] kCameraNames = {
      kFrontRightName, kFrontLeftName, kRearLeftName, kRearRightName
    };

    public static final Matrix<N3, N1> m_stateStdDevs =
        VecBuilder.fill(0.15, 0.15, 0.00001); // TODO
    public static final Matrix<N3, N1> m_visionStdDevs = VecBuilder.fill(.7, .7, 999999); // TODO
  }

  public static final class ShooterConstants {
    public static final int kTurretCanId = 29;
    public static final int kTurretMaxRange = 200;
    public static final double kFwdLimitSwitchOffset = 16.33;
    public static final double kRevLimitSwitchOffset = -16.33;
    public static final int kTurretMinRange = -200;
    public static final double kTurretMountingOffsetDegrees = 0.0;
    public static final Transform2d turretOffset =
        new Transform2d(
            Units.inchesToMeters(-5), Units.inchesToMeters(0), Rotation2d.fromDegrees(0));
    public static final double kTurretGearRatio = 52.5;
    public static final double kTurretMOI = 0.0722989441;

    // Generalization of updating the targets
    public static final double kLatencyCompensation = 0.1;

    public static final int kHoodCanId = 32;
    public static final double kHoodMaxAngle = 72.276537;
    public static final double kHoodMinAngle = 54.276537;
    public static final double kHoodZeroingSpeed = 0.1;

    public static final int kFlywheelLeaderMotorId = 30;
    public static final int kFlywheelFollowerMotorId = 31;

    public static final double kFlywheelDebounceTimeSeconds = 0.1;
    public static final double kTurretDebounceTimeSeconds = 0.1;
    public static final double kHoodDebounceTimeSeconds = 0.1;

    public static final double kShooterPositionTolerance =
        1; // degrees in which the relative encoder and setpoitn values can be off

    public static final double kTurretKV = 0.06; // TODO: tune
    public static final double kFFDeadbandDegrees = 30.0; // TODO: tune

    public static final class TurretSetpoints {
      public static final double kStow = 0;
    }

    public static final class HoodSetpoints {
      public static final double kStow = 15.2;
      public static final double kZeroOffsetDegrees = 0.0;
      public static final double kHoodVelocityTolerance = 0.5;
    }

    public static final class FlywheelSetpoints {
      public static final double kStow = 0.0;
      public static final double kStartSpeed = 1.0;
    }
  }

  public static final class DyeRotorConstants {
    public static final int kDyeRotorMotorCanID = 60;
    public static final int kDyeRotorFollowerMotorCanID = 61;
    public static final double kDyeRotorPower = 1.0;
  }

  public static final class ClimbConstants {
    public static final int kClimbMotorCanID = 45;

    // The absolute encoder is zeroed such that the "zero position" (straight down) is 20 degrees
    public static final double kZeroOffsetDegrees = 20;

    public static final double kPositionTolerance = 2.0;
    public static final double kDeployedSetpoint = 90 + kZeroOffsetDegrees;
    public static final double kClimbSetpoint = 150 + kZeroOffsetDegrees;
    public static final double kStowSetpoint = 0 + kZeroOffsetDegrees;
  }

  public static final class AutoAimConstants {
    // Aim at starting line since LUT is calibrated for the height of the hub
    public static final Translation2d kRedLeftTarget =
        new Translation2d(
            FieldConstants.fieldLength - FieldConstants.LinesVertical.starting,
            FieldConstants.fieldWidth - (FieldConstants.LinesHorizontal.leftTrenchOpenEnd - 0.6));
    public static final Translation2d kRedRightTarget =
        new Translation2d(
            FieldConstants.fieldLength - FieldConstants.LinesVertical.starting,
            FieldConstants.fieldWidth
                - (FieldConstants.LinesHorizontal.rightTrenchOpenStart + 0.6));
    public static final Translation2d kBlueLeftTarget =
        new Translation2d(
            FieldConstants.LinesVertical.starting,
            FieldConstants.LinesHorizontal.leftTrenchOpenEnd - 0.6);
    public static final Translation2d kBlueRightTarget =
        new Translation2d(
            FieldConstants.LinesVertical.starting,
            FieldConstants.LinesHorizontal.rightTrenchOpenStart + 0.6);
  }

  // For field constants
  public static boolean disableHAL = false;

  public static final class AgitationConstants {
    public static final double kStowDurationSeconds = 0.25;
    public static final double kExtendDurationSeconds = 0.5;
    public static final double kAgitateOutPower = -0.15;
    public static final double kAgitateInPower = 0.2;
  }

  public static final class IntakeConstants {

    public static final class RollerConstants {
      public static final int kIntakeRollerCanId = 10; // needs tuning
      public static final double kIntakeRollerPower = 1; // needs tuning
      public static final double kExtakeRollerPower = -1; // needs tuning
      public static final double kRollerStop = 0.0; // needs tuning
    }

    public static final class PivotConstants {
      public static final int kIntakePivotCanId = 9; // needs tuning
      public static final double kPivotkG = 0; // needs tuning
      public static final double kPivotStow = 90; // needs tuning
      public static final double kPivotExtend = 0; // needs tuning

      public static final double kPivotUpPower = 0.42;
      public static final double kPivotDownPower = -0.42;

      public static final int kPivotThreshold = 1; // needs tuning
      public static final int kPivotClear = 75; // needs tuning
      public static final double kPivotVelocityThreshold = 0.1;
    }
  }
}
