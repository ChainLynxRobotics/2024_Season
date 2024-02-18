package frc.robot.constants;

/**
 * Software/hardware constants (e.g. CAN IDs, gear ratios, field measurements, etc.). For software
 * configs @see RobotConfig
 */
public final class RobotConstants {
  public final class ShooterConstants {
    public static final int kRollerMotorLeftId = -1;
    public static final int kRollerMotorRightId = -1;
    public static final int kAngleMotorLeaderId = -1;
    public static final int kAngleMotorFollowerId = -1;
    public static final int kTopFlywheelMotorId = -1;
    public static final int kBottomFlywheelMotorId = -1;
    public static final int kShieldMotorId = -1;
    public static final double FlywheelDiameter = 0.0762;
    public static final double ShooterLength = 0.4064;
    public static final double Gravity = 9.81;
  }

  public final class Bindings {
    public static final int kAimSpeaker = 8;
    public static final int kAimAmp = 3;
    public static final int kChangeToManual = 6;
    public static final int kShoot = 1;
    public static final int kAimTrap = 2;
    public static final int kStowShooter = 4;
    public static final int kChangeToAutonomous = 5;
    public static final int kManualAngleSlider = 3;
  }
}
