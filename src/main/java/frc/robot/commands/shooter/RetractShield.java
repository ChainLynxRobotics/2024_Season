package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class RetractShield extends Command {
  private final Shooter m_shooter;

  public RetractShield(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.retractShield();
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShieldMotor();
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(ShooterConfig.kShieldExtendedPosition - m_shooter.getShieldPosition()) < ShooterConfig.kPositionError) {
      return true;
    }
    else {
      return false;
    }
  }
}
