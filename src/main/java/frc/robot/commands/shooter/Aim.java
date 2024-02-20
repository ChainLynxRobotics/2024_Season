package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class Aim extends Command {
  private final Shooter m_shooter;
  private final FieldElement m_type;

  public Aim(Shooter shooter, FieldElement type) {
    m_shooter = shooter;
    m_type = type;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  /**
   * Takes in distances to calculate shot, then shoots
   *
   * <p>Takes vertical height from constants Takes horizontal distance from vision Calculates angle
   * and velocity
   */
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // aim for speaker
    double desiredAngle = 0;
    switch (m_type) {
      case SPEAKER:
        desiredAngle = ShooterConfig.kSpeakerAngle;
        m_shooter.setAngle(desiredAngle);
      case AMP:
        desiredAngle = ShooterConfig.kAmpAngle;
        m_shooter.setAngle(desiredAngle);
        m_shooter.extendShield();
        // TODO actuate shield
      case TRAP:
        desiredAngle = ShooterConfig.kTrapAngle;
        m_shooter.setAngle(desiredAngle);
        m_shooter.extendShield();
      default:
    }

    // aim for amp

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_type == FieldElement.AMP || m_type == FieldElement.TRAP) {
      m_shooter.retractShield();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
