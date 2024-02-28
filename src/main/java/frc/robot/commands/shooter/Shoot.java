package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends Command {
  private final Shooter m_shooter;
  private double timer;
  private boolean m_reverse;

  public Shoot(Shooter shooter, boolean reverse) {
    m_shooter = shooter;
    m_reverse = reverse;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    timer = Timer.getFPGATimestamp();
    m_shooter.startFeedNote(m_reverse);
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
