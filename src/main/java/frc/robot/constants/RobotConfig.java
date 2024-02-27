package frc.robot.constants;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/**
 * Software config settings (e.g. max speed, PID values). For hardware constants @see
 * RobotConstants"
 */
public class RobotConfig {
  public enum AdjustType {
    up,
    down
  }

  public enum FieldElement {
    SPEAKER,
    AMP,
    TRAP
  }

  public static final class ShooterConfig {
    // Angle controller PID coefficients
    public static final double kAngleControlP = 0;
    public static final double kAngleControlI = 0;
    public static final double kAngleControlD = 0;
    public static final double kAngleControlFF = 0;
    public static final double kAngleControlIZone = 0;
    public static final double kAngleControlMinOutput = 0;
    public static final double kAngleControlMaxOutput = 0;

    // top Flywheel controller PID coefficients
    public static final double kTopFlywheelP = 0;
    public static final double kTopFlywheelI = 0;
    public static final double kTopFlywheelD = 0;
    public static final double kTopFlywheelFF = 0;
    public static final double kTopFlywheelIZone = 0;
    public static final double kTopFlywheelMinOutput = 0;
    public static final double kTopFlywheelMaxOutput = 0;

    // Smart dashboard Angle Controller keys
    public static final String kAngleControlPGainKey = "Angle Controller P Gain";
    public static final String kAngleControlIGainKey = "Angle Controller I Gain";
    public static final String kAngleControlDGainKey = "Angle Controller D Gain";
    public static final String kAngleControlFFGainKey = "Angle Controller FF Gain";
    public static final String kAngleControlIZoneKey = "Angle Controller I Zone";
    public static final String kAngleControlMinOutputKey = "Angle Controller Minimum output";
    public static final String kAngleControlMaxOutputKey = "Angle Controller Maximum output";

    // Smart dashboard top Flywheel Controller keys
    public static final String kTopFlywheelPGainKey = "Top Flywheel P Gain";
    public static final String kTopFlywheelIGainKey = "Top Flywheel I Gain";
    public static final String kTopFlywheelDGainKey = "Top Flywheel D Gain";
    public static final String kTopFlywheelFFGainKey = "Top Flywheel FF Gain";
    public static final String kTopFlywheelIZoneKey = "Top Flywheel I Zone";
    public static final String kTopFlywheelMinOutputKey = "Top Flywheel Minimum output";
    public static final String kTopFlywheelMaxOutputKey = "Top Flywheel Maximum output";

    // Roller Default Speed
    public static final double kRollerDefaultSpeed = 0;
    // Flywheel default speed
    public static final double kFlywheelDefaultRPM = 0;
    // Shield Extended position
    public static final double kShieldExtendedRotations = 124.140855612;
    // Shield Retracted position
    public static final double kShieldRetractedRotations = 0;
    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;
    public static final double kShieldExtendedPosition = 10; // TODO get correct value
    // Speaker height
    public static final double SpeakerHeight = 1.9812;
    public static final double AmpHeight = .46;
    public static final double ShooterHeight = 0.28575;
    public static final double TrapHeight = -1;

    public static final double SpeakerBillLength = 0.6604;

    public static final int shootButton = 1;

    public static final double kMaxFlywheelRPM = 11000;

    public static final double kShooterStowAngle = 0;

    public static final long kReleaseTime = 500;
    public static final Measure<Velocity<Angle>> kFlywheelError = Units.RPM.of(1);
    public static final Measure<Angle> kAngleError = Units.Degrees.of(0.5);
    public static final Measure<Angle> kSpeakerAngle = Units.Degrees.of(75);
    public static final Measure<Angle> kAmpAngle = Units.Degrees.of(109);
    public static final Measure<Angle> kTrapAngle = Units.Degrees.of(105);
    public static final Measure<Angle> kAdjustAmountDegrees = Units.Degrees.of(0.5);
  }

  public static class DriveConfig {
    public static class TranslateConfig {
      public static final String kPKey = "Vision Translate P";
      public static final String kIKey = "Vision Translate I";
      public static final String kDKey = "Vision Translate D";
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kTolerance = 1.0;
      public static final double minIntegral = 0;
      public static final double maxIntegral = 2;
    }

    public static class TurnConfig {
      public static final String kPKey = "Vision Turn P";
      public static final String kIKey = "Vision Turn I";
      public static final String kDKey = "Vision Turn D";
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kTolerance = 1.0;
      public static final double minIntegral = 0;
      public static final double maxIntegral = 8;
    }

    // 4.45 m/s max speed
    public static final double kMaxSpeedBase = 4.8;
    public static final double kMaxSpeedScaleFactor = 0.9;
    public static final double kMaxSpeedMetersPerSecond = kMaxSpeedBase * kMaxSpeedScaleFactor;

    public static final double kMaxAngularSpeedBase = Math.PI;
    public static final double kMaxAngularSpeedScaleFactor = 0.7;
    public static final double kMaxAngularSpeed =
        kMaxAngularSpeedBase * kMaxAngularSpeedScaleFactor; // radians per second

    public static final double kFrontLeftChassisAngularOffset = 0.0;
    public static final double kFrontRightChassisAngularOffset = 0.0;
    public static final double kBackLeftChassisAngularOffset = 0.0;
    public static final double kBackRightChassisAngularOffset = 0.0;
    // scaling factor for the alternative turning mode
    public static final int altTurnSmoothing = 20;
    public static final double HIGH_DIRECTION_SLEW_RATE = 500;
    public static final double MIN_ANGLE_SLEW_RATE = 0.45;
    public static final double MAX_ANGLE_SLEW_RATE = 0.85;
  }
}
