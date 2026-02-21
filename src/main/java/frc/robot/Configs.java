package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;

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
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .voltageCompensation(12);
      turretConfig
          .externalEncoder
          .positionConversionFactor(360)
          .inverted(false)
          .countsPerRevolution(8192);
      turretConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .pid(0.08, 0, 0)
          .outputRange(-1, 1);
        turretConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotorAndSetPosition)
        .forwardLimitSwitchPosition(ShooterConstants.kLimitSwitchOffset)
        .limitSwitchPositionSensor(FeedbackSensor.kAlternateOrExternalEncoder);

      hoodConfig
          .smartCurrentLimit(40)
          .idleMode(IdleMode.kBrake)
          .inverted(false)
          .voltageCompensation(12);
      hoodConfig.externalEncoder.positionConversionFactor(360).inverted(false);
      hoodConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
          .p(0.1)
          .d(0)
          .outputRange(-1, 1);

      flywheelConfigLeader
          .smartCurrentLimit(60)
          .idleMode(IdleMode.kCoast)
          .inverted(true)
          .voltageCompensation(12);

      flywheelConfigLeader
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.001)
          .outputRange(-1, 1)
          .feedForward
          .kV(0.00178);

      flywheelConfigFollower
          .idleMode(IdleMode.kCoast)
          .follow(ShooterConstants.kFlywheelLeaderMotorId, true);
    }
  }

  public static final class DyeRotor {
    public static final SparkFlexConfig dyeRotorConfig = new SparkFlexConfig();

    static {
      dyeRotorConfig
          .smartCurrentLimit(40) // needs tuning
          .idleMode(IdleMode.kBrake) // needs tuning
          .inverted(false) // needs tuning
          .voltageCompensation(12); // needs tuning
      dyeRotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // needs tuning
          .p(0.01) // needs tuning
          .d(0) // needs tuning
          .outputRange(-0.5, 0.5); // needs tuning
    }
  }

  public static final class Intake {
    public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
    public static final SparkFlexConfig rollerConfig = new SparkFlexConfig();

    static {
      pivotConfig
          .smartCurrentLimit(40) // needs tuning
          .idleMode(IdleMode.kBrake) // needs tuning
          .inverted(false) // needs tuning
          .voltageCompensation(12); // needs tuning
      pivotConfig
          .absoluteEncoder
          .positionConversionFactor(360)
          .inverted(false)
          .zeroCentered(true); // needs tuning
      pivotConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // needs tuning
          .p(0.01) // needs tuning
          .d(0) // needs tuning
          .outputRange(-1.0, 1); // needs tuning
    }

    static {
      rollerConfig
          .smartCurrentLimit(40) // needs tuning
          .idleMode(IdleMode.kBrake) // needs tuning
          .inverted(false) // needs tuning
          .voltageCompensation(12); // needs tuning
    }
  }
}
