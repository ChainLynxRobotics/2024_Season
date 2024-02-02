package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.utils.Vector;
import org.photonvision.PhotonUtils;

public class VisionTranslateCommand extends Command {

  // private VisionSubsystem vision;
  private Vision vision;
  private Drivetrain drive;
  private XboxController controller;
  private double kp = 0.2;
  private double ki = 10;
  private double kd = 0.05;

  private PIDController forwardController;

  public VisionTranslateCommand(Vision vision, Drivetrain drive, XboxController controller) {
    this.vision = vision;
    this.drive = drive;

    this.controller = controller;
    SmartDashboard.putNumber("kP", kp);
    SmartDashboard.putNumber("kI", ki);
    SmartDashboard.putNumber("kD", kd);
    forwardController = new PIDController(kp, ki, kd);

    addRequirements(vision, drive);

    forwardController.setIntegratorRange(0, 2);
  }

  @Override
  public void execute() {
    double forwardSpeed = -controller.getLeftY();

    if (vision.getHasTarget()) {
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              RobotConstants.VisionConstants.kCameraHeight,
              RobotConstants.VisionConstants.kTargetHeight,
              RobotConstants.VisionConstants.kCameraPitchRadians,
              Units.degreesToRadians(vision.getBestTarget().getPitch()));

      forwardSpeed = forwardController.calculate(range, 0);
    } else {
      forwardSpeed = 0;
    }

    drive.drive(
        new Vector(
            MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.kDriveDeadband),
            MathUtil.applyDeadband(controller.getRightX(), DriveConstants.kDriveDeadband)),
        new Vector(MathUtil.applyDeadband(forwardSpeed, DriveConstants.kDriveDeadband), 0),
        false,
        false);
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(forwardController.getPositionError()) < 1) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(new Vector(0, 0), new Vector(0, 0), false, false);
  }
}