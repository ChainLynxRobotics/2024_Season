package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.subsystems.climber.Climber;

public class Climb extends Command {
  private final Climber m_climber;

  public Climb(Climber climber) {
    m_climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_climber.setMotorSpeed(ClimberConfig.kDefaultSpeed);
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
