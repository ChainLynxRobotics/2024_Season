package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootSpeaker extends Command {
  private final Shooter m_shooter;

  public ShootSpeaker(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  /**
   * Takes in distances to calculate shot, then shoots
   *
   * <p>Takes vertical height from constants Takes horizontal distance from vision Calculates angle
   * and velocity
   */
  // TODO: implement kinematics
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredAngle = calculateAngle(ShooterConfig.billLenght, ShooterConfig.AmpHeight);
    m_shooter.setAngle(desiredAngle);
    double desiredRPM = convertToRPM(calculateVelocity(ShooterConfig.billLenght, ShooterConfig.SpeakerHeight));
    m_shooter.runFlywheel(
        desiredRPM);
    if (m_shooter.getCurrentRPM() >= desiredRPM && m_shooter.getCurrentAngle() < desiredAngle + 5 && m_shooter.getCurrentAngle() > desiredAngle - 2) {
      m_shooter.startFeedNote();
    }
    // make it shoot the thing
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

  public double addHeight(double theta) {
    double height = Math.sin(Math.toRadians(theta)) * ShooterConstants.ShooterLength;
    return height;
  }

  public double calculateVelocity(double targetX, double targetY) {
    targetY += addHeight(calculateAngle(targetX, targetY));
    double velocityY = Math.sqrt(2 * ShooterConstants.Gravity * targetY);
    double theta = calculateAngle(targetX, targetY);
    double velocity = velocityY / Math.sin(theta);
    return (velocity);
  }
  public double convertToRPM(double velocity) {
    // 0.0762 is diameter of flywheel
    double circumference = ShooterConstants.FlywheelDiameter * Math.PI;
    double rpm = velocity / circumference;
    return rpm;
  }
}
