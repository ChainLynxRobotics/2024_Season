package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
  private final Shooter m_shooter;
  private double timer;

  public Shoot(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    timer = Timer.getFPGATimestamp();
    m_shooter.startFeedNote();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFeedNote();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timer > RobotConfig.ShooterConfig.kReleaseTime;
  }
}
