package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class ActuateShield extends Command {
  private final Shooter m_shooter;
  private final boolean m_shieldState;
  private double initTime;

  public ActuateShield(Shooter shooter, boolean extend) {
    m_shooter = shooter;
    m_shieldState = extend;

    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double timeDiff = Timer.getFPGATimestamp() - initTime;
    if (timeDiff < ShooterConfig.kShieldTime) {
      m_shooter.setShield(m_shieldState);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShieldMotor();
  }

  @Override
  public boolean isFinished() {
    return m_shooter.getShieldStatus(m_shieldState);
  }
}
