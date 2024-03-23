package frc.robot.commands.shooter;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class PivotMove extends Command {
  private Shooter m_shooter;
  private double m_multiplier;
  private double startTime;

  public PivotMove(Shooter shooter, double multiplier) {
    m_shooter = shooter;
    m_multiplier = multiplier;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_shooter.setAngle(MutableMeasure.ofBaseUnits(160 * m_multiplier, Units.Rotations));
  }

  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) > 1;
  }

  @Override
  public void execute() {
    double ff =
        Math.cos(m_shooter.getCurrentAngle().in(Units.Radians)) * ShooterConfig.kAngleControlFF;
    m_shooter.setFF(ff);
  }
}
