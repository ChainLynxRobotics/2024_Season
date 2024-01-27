package frc.robot.constants;

/**
 * Software config settings (e.g. max speed, PID values). For hardware constants @see
 * RobotConstants"
 */
public class RobotConfig {
  public static final class IntakeConfig {
    // Roller PID coefficients
    public static final double kRollerP = 0;
    public static final double kRollerI = 0;
    public static final double kRollerD = 0;
    public static final double kRollerFF = 0;
    public static final double kRollerIZone = 0;
    public static final double kRollerMinOutput = 0;
    public static final double kRollerMaxOutput = 0;

    // Smart dashboard roller keys
    public static final String kRollerPGainKey = "Roller P Gain";
    public static final String kRollerIGainKey = "Roller I Gain";
    public static final String kRollerDGainKey = "Roller D Gain";
    public static final String kRollerFFGainKey = "Roller FF Gain";
    public static final String kRollerIZoneKey = "Roller I Zone";
    public static final String kRollerMinOutputKey = "Roller Minimum output";
    public static final String kRollerMaxOutputKey = "Roller Maximum output";

    // Smart dashboard test mode keys
    public static final String kTestCheck1Key = "Test Check 1";
    public static final String kTestCheck2Key = "Test Check 2";

    // Default Speed
    public static final double kDefaultSpeed = 0;

    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;
  }
}
