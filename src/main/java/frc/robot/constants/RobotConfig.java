package frc.robot.constants;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

/**
 * Software config settings (e.g. max speed, PID values). For hardware constants @see
 * RobotConstants"
 */
public class RobotConfig {
  public static final class ClimberConfig {
    public static final double kSetPointTolerance = 0.1;
    public static final double kStallInput = 0.02;
    public static final Measure<Distance> buddyClimbExtensionDiff =
        Units.Meters.of(Units.Inches.of(5).in(Units.Meters));
  }
}
