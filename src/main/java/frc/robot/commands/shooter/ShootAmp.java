package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants;
public class ShootAmp extends Command {
  private final Shooter m_shooter;

  public ShootAmp(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredAngle = calculateAngle(-1, RobotConfig.ShooterConfig.AmpHeight);
    m_shooter.setAngle(desiredAngle);
    double desiredRPM = convertToRPM(calculateVelocity(-1, RobotConfig.ShooterConfig.AmpHeight));
    m_shooter.runFlywheel(
        desiredRPM);
    if (m_shooter.getCurrentRPM() >= desiredRPM && m_shooter.getCurrentAngle() < desiredAngle + 5 && m_shooter.getCurrentAngle() > desiredAngle - 2) {
      m_shooter.startFeedNote();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double calculateAngle(double targetX, double targetY) {
    double theta = Math.toDegrees(Math.atan2(targetY, targetX));
    return theta;
  }

  public double convertToRPM(double velocity) {
    // 0.0762 is diameter of flywheel
    double circumference = RobotConstants.ShooterConstants.FlywheelDiameter * Math.PI;
    double rpm = velocity / circumference;
    return rpm;
  }
  public double calculateVelocity(double targetX, double targetY) {
    double velocityY = Math.sqrt(2 * RobotConstants.ShooterConstants.Gravity * targetY);
    double theta = calculateAngle(targetX, targetY);
    double velocity = velocityY / Math.sin(theta);
    return (velocity);
  }
}
