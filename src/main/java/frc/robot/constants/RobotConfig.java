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

    // top hood pid coefficients
    public static final double kTopHoodP = 0;
    public static final double kTopHoodI = 0;
    public static final double kTopHoodD = 0;
    public static final double kTopHoodFF = 0;
    public static final double kTopHoodIZone = 0;
    public static final double kTopHoodMinOutput = 0;
    public static final double kTopHoodMaxOutput = 0;

    // Smart dashboard top Hood Controller keys
    public static final String kTopHoodPGainKey = "Top Hood P Gain";
    public static final String kTopHoodIGainKey = "Top Hood I Gain";
    public static final String kTopHoodDGainKey = "Top Hood D Gain";
    public static final String kTopHoodFFGainKey = "Top Hood FF Gain";
    public static final String kTopHoodIZoneKey = "Top Hood I Zone";
    public static final String kTopHoodMinOutputKey = "Top Hood Minimum output";
    public static final String kTopHoodMaxOutputKey = "Top Hood Maximum output";

    // Smart dashboard test mode keys
    public static final String kTestCheck1Key = "Test Check 1";
    public static final String kTestCheck2Key = "Test Check 2";

    // Default Speed
    public static final double kDefaultSpeed = 0;

    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;
  }
}
