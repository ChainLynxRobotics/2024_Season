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

    // Hood controller PID coefficients
    public static final double kHoodP = 0;
    public static final double kHoodI = 0;
    public static final double kHoodD = 0;
    public static final double kHoodFF = 0;
    public static final double kHoodIZone = 0;
    public static final double kHoodMinOutput = 0;
    public static final double kHoodMaxOutput = 0;

    // Smart dashboard Hood Controller keys
    public static final String kHoodPGainKey = "Hood P Gain";
    public static final String kHoodIGainKey = "Hood I Gain";
    public static final String kHoodDGainKey = "Hood D Gain";
    public static final String kHoodFFGainKey = "Hood FF Gain";
    public static final String kHoodIZoneKey = "Hood I Zone";
    public static final String kHoodMinOutputKey = "Hood Minimum output";
    public static final String kHoodMaxOutputKey = "Hood Maximum output";

    // Smart dashboard test mode keys
    public static final String kTestCheck1Key = "Test Check 1";
    public static final String kTestCheck2Key = "Test Check 2";

    // Default Speed
    public static final double kDefaultSpeed = 0;

    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;
  }
}
