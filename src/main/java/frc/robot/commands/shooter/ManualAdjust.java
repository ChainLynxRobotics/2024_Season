package frc.robot.commands.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConfig.AdjustType;
import frc.robot.subsystems.shooter.Shooter;

public class ManualAdjust extends Command {
  private final Shooter m_shooter;
  private final AdjustType m_type;
  private Measure<Angle> desiredAngle;

  public ManualAdjust(Shooter shooter, AdjustType type) {
    m_shooter = shooter;
    m_type = type;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    switch (m_type) {
      case up:
        desiredAngle = m_shooter.getCurrentAngle().plus(ShooterConfig.kAdjustAmountDegrees);
        m_shooter.setAngle(desiredAngle);
        break;
      case down:
        desiredAngle = m_shooter.getCurrentAngle().minus(ShooterConfig.kAdjustAmountDegrees);
        m_shooter.setAngle(desiredAngle);
        break;
      default:
        desiredAngle = m_shooter.getCurrentAngle();
        m_shooter.setAngle(desiredAngle);
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return m_shooter.isAtAngleSetpoint(desiredAngle.magnitude());
  }
}
