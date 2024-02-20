package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
  private final Shooter m_shooter;

  public Shoot(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.startFeedNote();
    try {
      Thread.sleep(RobotConfig.ShooterConfig.kReleaseTime);
    } catch (InterruptedException e) {}
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFeedNote();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
