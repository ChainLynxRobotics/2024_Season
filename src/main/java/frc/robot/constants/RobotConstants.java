package frc.robot.constants;

/**
 * Software/hardware constants (e.g. CAN IDs, gear ratios, field measurements, etc.). For software
 * configs @see RobotConfig
 */
public final class RobotConstants {
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

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorJoystickPort = 1;

    public static final double kDriveDeadband = 0.06;
    public static final double kMagnitudeDeadband = 0.06;
    public static final double kDirectionSlewRate = 10; // radians per second
    public static final double kMagnitudeSlewRate = 90; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 90; // percent per second (1 = 100%)
  }
}
