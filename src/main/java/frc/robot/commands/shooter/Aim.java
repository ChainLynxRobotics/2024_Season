package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class Aim extends Command {
  private final Shooter m_shooter;
  private final Vision m_vision;
  private final FieldElement m_type;
  private double desiredAngle;

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

  // Called when the command is initially scheduled.
  /**
   * Takes in distances to calculate shot, then shoots
   *
   * <p>Takes vertical height from constants Takes horizontal distance from vision Calculates angle
   * and velocity
   */
  @Override
  public void initialize() {
    if (m_type == null) {
      if (m_vision.getHasTarget()) {
        double desiredAngle = m_vision.getBestTarget().getPitch();
        m_shooter.setAngle(desiredAngle);
      }
    } else {
      switch (m_type) {
        case AMP:
          m_shooter.setAngle(ShooterConfig.kAmpAngle);
          break;
        case SPEAKER:
          m_shooter.setAngle(ShooterConfig.kSpeakerAngle);
          break;
        case TRAP:
          m_shooter.setAngle(ShooterConfig.kTrapAngle);
          break;
        default:
          m_shooter.setAngle(0.0);
          break;

      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAngleMotor();
  }
}
