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

    // top Flywheel controller PID coefficients
    public static final double kTopFlywheelP = 0;
    public static final double kTopFlywheelI = 0;
    public static final double kTopFlywheelD = 0;
    public static final double kTopFlywheelFF = 0;
    public static final double kTopFlywheelIZone = 0;
    public static final double kTopFlywheelMinOutput = 0;
    public static final double kTopFlywheelMaxOutput = 0;

    // bottom Flywheel controller PID coefficients
    public static final double kBottomFlywheelP = 0;
    public static final double kBottomFlywheelI = 0;
    public static final double kBottomFlywheelD = 0;
    public static final double kBottomFlywheelFF = 0;
    public static final double kBottomFlywheelIZone = 0;
    public static final double kBottomFlywheelMinOutput = 0;
    public static final double kBottomFlywheelMaxOutput = 0;

    // Smart dashboard top Flywheel Controller keys
    public static final String kTopFlywheelPGainKey = "Top Flywheel P Gain";
    public static final String kTopFlywheelIGainKey = "Top Flywheel I Gain";
    public static final String kTopFlywheelDGainKey = "Top Flywheel D Gain";
    public static final String kTopFlywheelFFGainKey = "Top Flywheel FF Gain";
    public static final String kTopFlywheelIZoneKey = "Top Flywheel I Zone";
    public static final String kTopFlywheelMinOutputKey = "Top Flywheel Minimum output";
    public static final String kTopFlywheelMaxOutputKey = "Top Flywheel Maximum output";

    // Smart dashboard bottom Flywheel Controller keys
    public static final String kBottomFlywheelPGainKey = "Bottom Flywheel P Gain";
    public static final String kBottomFlywheelIGainKey = "Bottom Flywheel I Gain";
    public static final String kBottomFlywheelDGainKey = "Bottom Flywheel D Gain";
    public static final String kBottomFlywheelFFGainKey = "Bottom Flywheel FF Gain";
    public static final String kBottomFlywheelIZoneKey = "Bottom Flywheel I Zone";
    public static final String kBottomFlywheelMinOutputKey = "Bottom Flywheel Minimum output";
    public static final String kBottomFlywheelMaxOutputKey = "Bottom Flywheel Maximum output";

    // Smart dashboard test mode keys
    public static final String kTestCheck1Key = "Test Check 1";
    public static final String kTestCheck2Key = "Test Check 2";

    // Default Speed
    public static final double kDefaultSpeed = 0;

    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;
  }
}
