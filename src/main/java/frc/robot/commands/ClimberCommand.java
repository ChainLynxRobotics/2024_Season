package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.subsystems.climber.Climber;
import frc.robot.constants.RobotConstants;

public class ClimberCommand extends Command {
  private final Climber m_climber;
  private final double m_setpoint;

  public ClimberCommand(Climber climber, double setpoint) {
    m_climber = climber;
    m_setpoint = setpoint;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    m_climber.setSetpoint(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.setMotorSpeed(ClimberConfig.kStallInput);
  }

  @Override
  public boolean isFinished() {
    double error =
        Climber.metersToRotations(m_setpoint) - m_climber.getEncoderPosition();
    return Math.abs(error) <= RobotConstants.ClimberConstants.kSetPointTolerance;
  }
}
