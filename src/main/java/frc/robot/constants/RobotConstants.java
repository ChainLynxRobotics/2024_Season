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

    public static final double kClimberP = 0;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final double kClimberMotorRadius = 0.003175;
    public static final double kClimberIZone = 0;
    public static final double kClimberFeedForward = 0;
    public static final double kClimberMaxOutput = 0;
    public static final double kClimberMinOutput = 0;
  }
}
