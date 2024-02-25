package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
// done
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class StowShooter extends Command {
  private final Shooter m_shooter;

  public StowShooter(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setAngle(Units.Degrees.of(ShooterConfig.kShooterStowAngle));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
