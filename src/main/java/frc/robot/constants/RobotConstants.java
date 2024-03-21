package frc.robot.constants;

/**
 * Software/hardware constants (e.g. CAN IDs, gear ratios, field measurements, etc.). For software
 * configs @see RobotConfig
 */
public final class RobotConstants {
  public static final class Bindings {
    public static final int kLeftClimberUp = 11;
    public static final int kLeftClimberDown = 12;
    public static final int kRightClimberUp = 13;
    public static final int kRightClimberDown = 14;
    public static final int kBothClimbersUp = 11;
    public static final int kBothClimbersDown = 11;
  }

  public static final class ClimberConstants {
    public static final int kClimberLeaderID = 11;
    public static final int kClimberFollowerID = 12;

    public static final double kClimberP = 0.1;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;
    public static final double kClimberMotorRadius = 0.003175;
    public static final double kClimberIZone = 0.001;
    public static final double kClimberFeedForward = 0;
    public static final double kClimberMaxOutput = 1;
    public static final double kClimberMinOutput = -1;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorJoystickPort = 1;
  }
}
