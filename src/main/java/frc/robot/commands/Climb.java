package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.subsystems.climber.Climber;

public class Climb extends Command {
  private final Climber m_climber;
  private boolean m_reverse;

  public Climb(Climber climber, boolean reverse) {
    m_climber = climber;
    m_reverse = reverse;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    int multiplier = m_reverse ? -1 : 1;
    m_climber.setMotorSpeed(ClimberConfig.kDefaultSpeed*multiplier);
  }


  @Override
  public void end(boolean interrupted) {
    m_climber.setMotorSpeed(ClimberConfig.kStallInput);
  }

  @Override
  public boolean isFinished() {
    return m_climber.getLeaderEncoderPosition() > ClimberConfig.kUpperRotSoftStop - ClimberConfig.kStopMargin;
  }
}
