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
        desiredAngle =
            m_shooter.calculateAngle(ShooterConfig.billLength, ShooterConfig.SpeakerHeight);
        m_shooter.setAngle(desiredAngle);
      case AMP:
        desiredAngle = m_shooter.calculateAngle(-1, ShooterConfig.AmpHeight);
        m_shooter.setAngle(desiredAngle);
        // TODO actuate shield
      case TRAP:
      default:
    }

    if (Math.abs(m_shooter.getCurrentAngle() - desiredAngle) <= 1) {
      m_shooter.startFeedNote();
    }

    // aim for amp

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
