package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.subsystems.climber.Climber;

public class Climb extends Command {
  private final Climber m_climber;
  private double m_setpoint;

  public Climb(Climber climber, double setpoint) {
    m_climber = climber;
    m_setpoint = setpoint;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    m_climber.setSetpoint(m_climber.getLeaderPidController(), m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.setMotorSpeed(ClimberConfig.kStallInput);
  }

  @Override
  public boolean isFinished() {
    double error = Climber.metersToRotations(m_setpoint) - m_climber.getLeaderEncoderPosition();
    return Math.abs(error) <= ClimberConfig.kSetPointTolerance;
  }
}
