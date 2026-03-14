package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
          .feedForward
          .kV(drivingVelocityFeedForward);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class Shooter {
    public static final SparkFlexConfig turretConfig = new SparkFlexConfig();
    public static final SparkFlexConfig hoodConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelConfigLeader = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelConfigFollower = new SparkFlexConfig();

    static {
      turretConfig
          .smartCurrentLimit(20)
          .idleMode(IdleMode.kCoast)
          .inverted(true)
          .voltageCompensation(12);
      turretConfig
          .externalEncoder
          .positionConversionFactor(360.0 / 6.25)
          .inverted(true)
          .countsPerRevolution(8192);
      turretConfig.encoder.positionConversionFactor(360.0 / (6.25 * 25));
      turretConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .pid(0.05, 0, 0)
          .outputRange(-1, 1);
      turretConfig
          .limitSwitch
          .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotorAndSetPosition)
          .forwardLimitSwitchPosition(ShooterConstants.kFwdLimitSwitchOffset)
          .limitSwitchPositionSensor(FeedbackSensor.kAlternateOrExternalEncoder);
      turretConfig
          .limitSwitch
          .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotorAndSetPosition)
          .reverseLimitSwitchPosition(ShooterConstants.kRevLimitSwitchOffset)
          .limitSwitchPositionSensor(FeedbackSensor.kAlternateOrExternalEncoder);
      turretConfig
          .softLimit
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(ShooterConstants.kTurretMaxRange)
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(ShooterConstants.kTurretMinRange);

      hoodConfig
          .smartCurrentLimit(20)
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .voltageCompensation(12);
      hoodConfig
          .externalEncoder
          .countsPerRevolution(8192)
          .positionConversionFactor(16.363636)
          .inverted(false);
      hoodConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .p(0.1)
          .outputRange(-0.33, 0.33) // TODO(jan): Tune
          .feedForward
          .kS(0.175);

      flywheelConfigLeader
          .smartCurrentLimit(60)
          .idleMode(IdleMode.kCoast)
          .inverted(false)
          .voltageCompensation(12);

      flywheelConfigLeader.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

      flywheelConfigLeader
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(Robot.isReal() ? 0.0001 : 0.001)
          .outputRange(-1, 1)
          .feedForward
          .kV(Robot.isReal() ? 0.00185 : 0.00178);

      flywheelConfigFollower
          .idleMode(IdleMode.kCoast)
          .follow(ShooterConstants.kFlywheelLeaderMotorId, true);
    }
  }

  public static final class DyeRotor {
    public static final SparkFlexConfig dyeRotorConfig = new SparkFlexConfig();

    static {
      dyeRotorConfig
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kCoast)
          .inverted(true)
          .voltageCompensation(12);
      dyeRotorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
      dyeRotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.0001)
          .outputRange(-0.5, 0.5)
          .feedForward
          .kV(0.00185);
    }
  }

  public static final class Intake {
    public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
    public static final SparkFlexConfig rollerConfig = new SparkFlexConfig();

    static {
      pivotConfig
          .smartCurrentLimit(60)
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .voltageCompensation(12);
      pivotConfig.encoder.positionConversionFactor(360.0 / 75);

      // TODO: Tune PID
      pivotConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.1)
          .outputRange(-1, 1)
          .feedForward
          .kCos(0)
          .kCosRatio(1);
    }

    static {
      rollerConfig
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .voltageCompensation(12);
    }
  }

  public static final class Climb {
    public static final SparkFlexConfig climbConfig = new SparkFlexConfig();

    static {
      climbConfig
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .voltageCompensation(12);
      climbConfig.absoluteEncoder.positionConversionFactor(360).inverted(true);
      climbConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .p(0.05)
          .d(0.5)
          .outputRange(-0.5, 0.5)
          .feedForward
          .kS(0.115);

      // Faster slot for deploy/stow
      climbConfig
          .closedLoop
          .p(0.05, ClosedLoopSlot.kSlot1)
          .d(0.5, ClosedLoopSlot.kSlot1)
          .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
          .feedForward
          .kS(0.115, ClosedLoopSlot.kSlot1);
    }
  }
}
