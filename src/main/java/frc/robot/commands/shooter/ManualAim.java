package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ManualAim extends Command {
  private final Shooter m_shooter;
  private final DoubleSupplier m_angle;

  public ManualAim(Shooter shooter, DoubleSupplier launchAngle) {
    m_shooter = shooter;
    m_angle = launchAngle;
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
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // its weird were adjusting for 0-120
    double desiredAngle = -m_angle.getAsDouble() + 1 / 2 * 120;
    m_shooter.setAngle(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAngleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
