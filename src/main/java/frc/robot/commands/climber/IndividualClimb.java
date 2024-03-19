package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class IndividualClimb extends Command {
  private Climber m_climber;
  private boolean m_isRight;
  private boolean m_reverse;

  public IndividualClimb(Climber climber, boolean isRight, boolean reverse) {
    m_climber = climber;
    m_isRight = isRight;
    m_reverse = reverse;

    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    if (m_isRight) {
      m_climber.setLeader(m_reverse);
    } else {
      m_climber.setFollower(m_reverse);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (m_isRight) {
      m_climber.stopLeader();
    } else {
      m_climber.stopFollower();
    }
  }
}
