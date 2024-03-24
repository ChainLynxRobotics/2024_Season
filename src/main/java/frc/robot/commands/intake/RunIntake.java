package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
  private final Intake m_intake;
  private final Indexer m_indexer;
  private boolean m_reversed;

  public RunIntake(Intake intake, Indexer indexer, boolean reversed) {
    m_intake = intake;
    m_indexer = indexer;
    m_reversed = reversed;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    int multiplier = m_reversed ? -1 : 1;
    m_intake.run(RobotConfig.IntakeConfig.kDefaultSpeed * multiplier);
    m_indexer.startFeedNote(m_reversed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_indexer.stopFeedNote();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
