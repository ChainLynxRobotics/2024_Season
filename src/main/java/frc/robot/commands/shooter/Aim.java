package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class Aim extends Command {
  private final Shooter m_shooter;
  private final Vision m_vision;

  public Aim(Shooter shooter, Vision eyes) {
    m_shooter = shooter;
    m_vision = eyes;
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getHasTarget()) {
      double desiredAngle = m_vision.getBestTarget().getPitch();
      m_shooter.setAngle(desiredAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAngleMotor();
  }
}
