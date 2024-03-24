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
      m_climber.setLeft(m_reverse);
    } else {
      m_climber.setRight(m_reverse);
    }
  }

  @Override
  public void execute() {
    System.out.println("one climb");
  }

  @Override
  public void end(boolean interrupted) {
    if (m_isRight) {
      m_climber.stopLeft();
    } else {
      m_climber.stopRight();
    }
  }
}
