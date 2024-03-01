package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.intake.Intake;

public class Shoot extends Command {
  private final Intake m_intake;
  private double timer;

  public Shoot(Intake intake) {
    m_intake = intake;

    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    timer = Timer.getFPGATimestamp();
    m_intake.startFeedNote(true);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopFeedNote();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timer > RobotConfig.ShooterConfig.kReleaseTime;
  }
}
