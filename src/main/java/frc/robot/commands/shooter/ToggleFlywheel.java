package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class ToggleFlywheel extends Command {
  private Shooter m_shooter;

  public ToggleFlywheel(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (Math.abs(m_shooter.getCurrentRPM()) < 5) {
      m_shooter.runFlywheel(ShooterConfig.kFlywheelDefaultRPM);
    } else {
      m_shooter.stopFlywheel();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
