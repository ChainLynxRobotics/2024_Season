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

    // Angle PID coefficents
    public static final double kAngleP = 0;
    public static final double kAngleI = 0;
    public static final double kAngleD = 0;
    public static final double kAngleFF = 0;
    public static final double kAngleIZone = 0;
    public static final double kAngleMinOutput = 0;
    public static final double kAngleMaxOutput = 0;

    // Smart dashboard roller keys
    public static final String kRollerPGain = "Roller P Gain";
    public static final String kRollerIGain = "Roller I Gain";
    public static final String kRollerDGain = "Roller D Gain";
    public static final String kRollerFFGain = "Roller FF Gain";
    public static final String kRollerIZoneKey = "Roller I Zone";
    public static final String kRollerMinOutputKey = "Roller Minimum output";
    public static final String kRollerMaxOutputKey = "Roller Maximum output";

    // Smart dashboard angle keys
    public static final String kAnglePGain = "Angle P Gain";
    public static final String kAngleIGain = "Angle I Gain";
    public static final String kAngleDGain = "Angle D Gain";
    public static final String kAngleFFGain = "Angle FF Gain";
    public static final String kAngleIZoneKey = "Angle I Zone";
    public static final String kAngleMinOutputKey = "Angle Minimum output";
    public static final String kAngleMaxOutputKey = "Angle Maximum output";

    // Default Speed
    public static final double kDefaultSpeed = 0;
  }
}
