package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.AdjustType;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.constants.RobotConfig.*;

public class ManualAdjust extends Command {
  private final Shooter m_shooter;
  private final AdjustType m_type;
  private double desiredAngle;

  public ManualAdjust(Shooter shooter, AdjustType type) {
    m_shooter = shooter;
    m_type = type;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    switch (m_type) {
      case up:
        desiredAngle = m_shooter.getCurrentAngle() + ShooterConfig.kAdjustAmountDegrees;
        m_shooter.setAngle(desiredAngle);
        break;
      case down:
        desiredAngle = m_shooter.getCurrentAngle() - ShooterConfig.kAdjustAmountDegrees;
        m_shooter.setAngle(desiredAngle);
        break;
      default:
        desiredAngle = m_shooter.getCurrentAngle();
        m_shooter.setAngle(desiredAngle);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // TODO: put method from subsystem
    return false;
  }

}
