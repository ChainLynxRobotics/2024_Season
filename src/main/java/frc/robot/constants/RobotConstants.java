package frc.robot.constants;

/**
 * Software/hardware constants (e.g. CAN IDs, gear ratios, field measurements, etc.). For software
 * configs @see RobotConfig
 */
public final class RobotConstants {
  public static final class ClimberConstants {
    public static final int CLIMBER_CONTROLLER_ID1 = -1;
    public static final int CLIMBER_CONTROLLER_ID2 = -1;

    public static final double kSetPointTolerance = 0.01;

    public static final String kPGainKey = "Climber P Gain",
        kIGainKey = "Climber I Gain",
        kDGainKey = "Climber D Gain",
        kIZoneKey = "Climber I Zone",
        kFeedForwardKey = "Climber Feed Forward",
        kMaxOutputKey = "Climber Max Output",
        kMinOutputKey = "Climber Min Output",
        kSetRotationKey = "Set Rotations",
        kSetPointKey = "Setpoint",
        kProcessVariableKey = "Process Variable";
  }
}
