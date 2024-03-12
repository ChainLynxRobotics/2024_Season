package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.subsystems.climber.Climber;

public class BuddyClimb extends Command {
  private final Climber m_climber;
  private double m_setpoint;
  private boolean m_leftClimb;

  public BuddyClimb(Climber climber, double setpoint, boolean leftClimb) {
    m_climber = climber;
    m_setpoint = setpoint;
    m_leftClimb = leftClimb;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    double leftSetpoint, rightSetpoint = 0;
    if (m_leftClimb) {
      leftSetpoint = m_setpoint + ClimberConfig.buddyClimbExtensionDiff.magnitude();
      rightSetpoint = m_setpoint;
    } else {
      leftSetpoint = m_setpoint;
      rightSetpoint = m_setpoint + ClimberConfig.buddyClimbExtensionDiff.magnitude();
    }
    m_climber.setSetpoint(m_climber.getLeaderPidController(), leftSetpoint);
    m_climber.setSetpoint(m_climber.getFollowerPidController(), rightSetpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.resetFollower();
    m_climber.setMotorSpeed(ClimberConfig.kStallInput);
  }

  @Override
  public boolean isFinished() {
    double leaderError =
        Climber.metersToRotations(m_setpoint) - m_climber.getLeaderEncoderPosition();
    double followerError =
        Climber.metersToRotations(m_setpoint) - m_climber.getFollowerEncoderPosition();
    double accError = leaderError + followerError;
    return Math.abs(accError) <= ClimberConfig.kSetPointTolerance;
  }
}
