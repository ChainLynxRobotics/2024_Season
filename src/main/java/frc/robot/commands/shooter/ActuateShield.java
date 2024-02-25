package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class ActuateShield extends Command {
  private final Shooter m_shooter;
  private final boolean m_shieldState;

  public ActuateShield(Shooter shooter, boolean extend) {
    m_shooter = shooter;
    m_shieldState = extend;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    if (m_shieldState) {
      m_shooter.setShieldPosition(ShooterConfig.kShieldExtendedRotations);
    } else {
      m_shooter.setShieldPosition(ShooterConfig.kShieldRetractedRotations);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShieldMotor();
  }

  @Override
  public boolean isFinished() {
    // if we want the shield to be out, return true if that is the status
    if (m_shieldState) {
      return m_shooter.getShieldStatus();
    } else {
      return !m_shooter.getShieldStatus();
    }
  }
}
