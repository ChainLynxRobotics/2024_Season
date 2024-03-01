package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class Shoot extends Command {
  private final Indexer m_indexer;
  private boolean m_reverse;

  public Shoot(Indexer indexer, boolean reverse) {
    m_indexer = indexer;

    addRequirements(m_indexer);
  }

  @Override
  public void initialize() {
    m_indexer.startFeedNote(m_reverse);
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stopFeedNote();
  }
}
