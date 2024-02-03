package frc.robot.constants;

// variable things like default speeds
public class RobotConfig {

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
