package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.units.Units;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.DriveConstants.SwerveModuleConstants;

// variable things like default speeds
public class RobotConfig {

  public static class DriveConfig {
    public static final String kSlewRateTranslationMagOutput = "translation magnitude output";
    public static final String kSlewRateTranslationDirRadOutput = "translation dir rad";

    public static final HolonomicPathFollowerConfig kPathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                SwerveModuleConstants.kDrivingP,
                SwerveModuleConstants.kDrivingI,
                SwerveModuleConstants.kDrivingD),
            new PIDConstants(
                SwerveModuleConstants.kTurningP,
                SwerveModuleConstants.kTurningI,
                SwerveModuleConstants.kTurningD),
            SwerveModuleConstants.kMaxModuleSpeed.in(Units.MetersPerSecond),
            DriveConstants.kWheelBaseRadius.in(Units.Meters),
            new ReplanningConfig());

    // 4.45 m/s max speed
    public static final double kMaxSpeedBase = 4.8;
    public static final double kMaxSpeedScaleFactor = 0.9;
    public static final double kMaxSpeedMetersPerSecond = kMaxSpeedBase * kMaxSpeedScaleFactor;

    public static final double kMaxAngularSpeedBase = Math.PI;
    public static final double kMaxAngularSpeedScaleFactor = 0.7;
    public static final double kMaxAngularSpeed =
        kMaxAngularSpeedBase * kMaxAngularSpeedScaleFactor; // radians per second

    public static final double kFrontLeftChassisAngularOffset = 0.0;
    public static final double kFrontRightChassisAngularOffset = 0.0;
    public static final double kBackLeftChassisAngularOffset = 0.0;
    public static final double kBackRightChassisAngularOffset = 0.0;
    // scaling factor for the alternative turning mode
    public static final int altTurnSmoothing = 20;
    public static final double HIGH_DIRECTION_SLEW_RATE = 500;
    public static final double MIN_ANGLE_SLEW_RATE = 0.45 * Math.PI;
    public static final double MAX_ANGLE_SLEW_RATE = 0.85 * Math.PI;
  }

  public static final class IntakeConfig {
    // TODO: update intake motors default speed
    // In percentage output
    public static final double kDefaultSpeed = 0;

    // TODO: update timeout time (in seconds)
    public static final double kRunIntakeTimeoutSecs = 0;
  }
}
