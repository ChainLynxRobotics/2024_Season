package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.RobotContainer;

public class ShootSpeakerManual extends Command {
  private final Shooter m_shooter;
  private final BooleanSupplier m_index;
  private final DoubleSupplier m_angle;

  public ShootSpeakerManual(Shooter shooter, BooleanSupplier indexing, DoubleSupplier launchAngle) {
    m_shooter = shooter;
    m_index = indexing;
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
  // TODO: implement kinematics
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // converted from joystick axis range to rotations
    m_shooter.setAngle(m_angle.getAsDouble() / 4);
    double desiredRPM = ShooterConfig.kMaxFlywheelRPM;
    m_shooter.runFlywheel(
        desiredRPM);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAngleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_index.getAsBoolean()) {
        return true;
    }
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
