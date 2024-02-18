package frc.robot.constants;

/**
 * Software config settings (e.g. max speed, PID values). For hardware constants @see
 * RobotConstants"
 */
public class RobotConfig {
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

    // shield controller PID coefficients
    public static final double kShieldP = 0;
    public static final double kShieldI = 0;
    public static final double kShieldD = 0;
    public static final double kShieldFF = 0;
    public static final double kShieldIZone = 0;
    public static final double kShieldMinOutput = 0;
    public static final double kShieldMaxOutput = 0;

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

    // Smart dashboard shield Controller keys
    public static final String kShieldPGainKey = "Shield P Gain";
    public static final String kShieldIGainKey = "Shield I Gain";
    public static final String kShieldDGainKey = "Shield D Gain";
    public static final String kShieldFFGainKey = "Shield FF Gain";
    public static final String kShieldIZoneKey = "Shield I Zone";
    public static final String kShieldMinOutputKey = "Shield Minimum output";
    public static final String kShieldMaxOutputKey = "Shield Maximum output";

    // Roller Default Speed
    public static final double kRollerDefaultSpeed = 0;

    // Flywheel default speed
    public static final double kFlywheelDefaultRPM = 0;

    // Shield Extended position
    public static final double kShieldExtendedPosition = 0;

    // Shield Retracted position
    public static final double kShieldRetractedPosition = 0;

    // Timeout time (in seconds)
    public static final double kRunIntakeTimeoutTime = 0;

    // Speaker height
    public static final double SpeakerHeight = 1.9812 - 0.28575;

    public static final double AmpHeight = .46 - 0.28575;

    public static final double billLength = 0.6604;

    public static final int shootButton = 1;

    public static final double kMaxFlywheelRPM = 11000;

    public static final double kShooterStowAngle = 0;
  }
}
