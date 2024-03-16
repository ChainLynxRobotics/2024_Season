package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.SwerveModuleConstants;

/**
 * Software config settings (e.g. max speed, PID values). For hardware constants @see
 * RobotConstants"
 */
public class RobotConfig {
  public static final class ClimberConfig {
    public static final double kDefaultSpeed = 0.4;
    public static final double kStallInput = 0.02;
    public static final double kUpperRotSoftStop = 5000;
    public static final double kStopMargin = 10;
    public static final boolean kInverted = true;
    public static final Measure<Distance> buddyClimbExtensionDiff =
        Units.Meters.of(Units.Inches.of(5).in(Units.Meters));
  }

  public static class DriveConfig {
    public static class TranslateConfig {
      public static final String kPKey = "Vision Translate P";
      public static final String kIKey = "Vision Translate I";
      public static final String kDKey = "Vision Translate D";
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kTolerance = 1.0;
      public static final double minIntegral = 0;
      public static final double maxIntegral = 2;
    }

    public static class TurnConfig {
      public static final String kPKey = "Vision Turn P";
      public static final String kIKey = "Vision Turn I";
      public static final String kDKey = "Vision Turn D";
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kTolerance = 1.0;
      public static final double minIntegral = 0;
      public static final double maxIntegral = 8;
    }

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
            SwerveModuleConstants.kMaxModuleSpeed,
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
}
