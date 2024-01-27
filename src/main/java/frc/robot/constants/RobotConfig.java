package frc.robot.constants;

/**
 * Software config settings (e.g. max speed, PID values). For hardware constants @see
 * RobotConstants"
 */
public class RobotConfig {
  public static final class ShooterConfig {
    // Angle controller PID coefficients
    public static final double kAngleControlP = 0;
    public static final double kAngleControlI = 0;
    public static final double kAngleControlD = 0;
    public static final double kAngleControlFF = 0;
    public static final double kAngleControlIZone = 0;
    public static final double kAngleControlMinOutput = 0;
    public static final double kAngleControlMaxOutput = 0;

    // Smart dashboard Angle Controller keys
    public static final String kAngleControlPGainKey = "Angle Controller P Gain";
    public static final String kAngleControlIGainKey = "Angle Controller I Gain";
    public static final String kAngleControlDGainKey = "Angle Controller D Gain";
    public static final String kAngleControlFFGainKey = "Angle Controller FF Gain";
    public static final String kAngleControlIZoneKey = "Angle Controller I Zone";
    public static final String kAngleControlMinOutputKey = "Angle Controller Minimum output";
    public static final String kAngleControlMaxOutputKey = "Angle Controller Maximum output";

    // Flywheel controller PID coefficients
    public static final double kFlywheelP = 0;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelFF = 0;
    public static final double kFlywheelIZone = 0;
    public static final double kFlywheelMinOutput = 0;
    public static final double kFlywheelMaxOutput = 0;

    // Smart dashboard Flywheel Controller keys
    public static final String kFlywheelPGainKey = "Flywheel P Gain";
    public static final String kFlywheelIGainKey = "Flywheel I Gain";
    public static final String kFlywheelDGainKey = "Flywheel D Gain";
    public static final String kFlywheelFFGainKey = "Flywheel FF Gain";
    public static final String kFlywheelIZoneKey = "Flywheel I Zone";
    public static final String kFlywheelMinOutputKey = "Flywheel Minimum output";
    public static final String kFlywheelMaxOutputKey = "Flywheel Maximum output";

    // Smart dashboard test mode keys
    public static final String kTestCheck1Key = "Test Check 1";
    public static final String kTestCheck2Key = "Test Check 2";

    // Default Speed
    public static final double kDefaultSpeed = 0;

    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;
  }
}
