package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants;
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
    m_shooter.setAngle(calculateAngle(-1, RobotConfig.ShooterConfig.SpeakerHeight));
    m_shooter.runFlywheel(
        convertToRPM(calculateVelocity(-1, RobotConfig.ShooterConfig.SpeakerHeight)),
        calculateVelocity(-1, RobotConfig.ShooterConfig.SpeakerHeight));
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
    double time = Math.sqrt(2 * targetY / 9.8);
    double xVelocity = targetX / time;
    double yVelocity = 9.8 * time;
    double theta = Math.toDegrees(Math.atan2(yVelocity, xVelocity));
    return theta;
  }

  public double calculateVelocity(double targetX, double targetY) {
    double time = Math.sqrt(2 * targetY / 9.8);
    double xVelocity = targetX / time;
    double yVelocity = 9.8 * time;
    double velocity = Math.hypot(xVelocity, yVelocity);
    return velocity;
  }

  public double calculateTotalDragForce(double velocity, double angle) {
    // Convert angle to radians
    double thetaRad = Math.toRadians(angle);
    // trust
    double cd = 0.25;
    double rho = 1.869 * Math.pow(10, -5);
    // help this is so wrong i dont even know
    double area = 36 * 5 / 100;
    // Resolve velocity vector into horizontal and vertical components
    double vx = velocity * Math.cos(thetaRad);
    double vy = velocity * Math.sin(thetaRad);
    // Calculate drag force components
    double dragForceX = 0.5 * cd * area * rho * vx * vx;
    double dragForceY = 0.5 * cd * area * rho * vy * vy;
    // Calculate total drag force
    double totalDragForce = Math.hypot(dragForceX, dragForceY);
    return (totalDragForce);
  }

  public double calculateFinalVelocity(double targetX, double targetY) {
    double targetVelocity = calculateVelocity(targetX, targetY);
    double theta = calculateAngle(targetX, targetY);
    double finalVelocity = targetVelocity;
    while (finalVelocity - calculateTotalDragForce(finalVelocity, theta) < targetVelocity - 0.25) {
      finalVelocity = finalVelocity + calculateTotalDragForce(finalVelocity, theta);
    }
    return finalVelocity;
  }

  public double convertToRPM(double velocity) {
    // 0.0762 is diameter of flywheel
    double circumference = RobotConstants.ShooterConstants.FlywheelDiameter * Math.PI;
    double rpm = velocity / circumference;
    return rpm;
  }
}
