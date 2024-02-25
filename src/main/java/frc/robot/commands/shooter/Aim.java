package frc.robot.commands.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class Aim extends Command {
  private final Shooter m_shooter;
  private final Vision m_vision;
  private final FieldElement m_type;
  private Measure<Angle> desiredAngle;

  public Aim(Shooter shooter, FieldElement type) {
    m_shooter = shooter;
    m_vision = new Vision();
    m_type = type;

    addRequirements(m_shooter);
  }

  public Aim(Shooter shooter, Vision eyes) {
    m_shooter = shooter;
    m_vision = eyes;
    m_type = null;
    addRequirements(m_shooter, m_vision);
  }

  @Override
  public void initialize() {
    if (m_type == null) {
      if (m_vision.getHasTarget()) {
        Measure<Angle> desiredAngle = Units.Degrees.of(m_vision.getBestTarget().getPitch());
        m_shooter.setAngle(desiredAngle);
      }
    } else {
      switch (m_type) {
        case AMP:
          desiredAngle = ShooterConfig.kAmpAngle;
          m_shooter.setShieldPosition(ShooterConfig.kShieldExtendedRotations);
          break;
        case SPEAKER:
          desiredAngle = ShooterConfig.kSpeakerAngle;
          break;
        case TRAP:
          desiredAngle = ShooterConfig.kTrapAngle;
          m_shooter.setShieldPosition(ShooterConfig.kShieldRetractedRotations);
          break;
        default:
          desiredAngle = Units.Degrees.of(0);
          break;
      }
      m_shooter.setAngle(desiredAngle);
    }
  }

  public boolean isFinished() {
    return m_shooter.isAtAngleSetpoint(desiredAngle.magnitude())
    && ((m_type == FieldElement.AMP || m_type == FieldElement.TRAP) && m_shooter.getShieldStatus()); //check if shield is extended
  }
}
