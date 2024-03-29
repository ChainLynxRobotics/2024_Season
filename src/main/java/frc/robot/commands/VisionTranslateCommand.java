package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.DriveConfig.TranslateConfig;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.utils.Vector;

public class VisionTranslateCommand extends Command {

  private Vision vision;
  private Drivetrain drive;
  private XboxController controller;

  private PIDController forwardController;

  public VisionTranslateCommand(Vision vision, Drivetrain drive, XboxController controller) {
    this.vision = vision;
    this.drive = drive;

    this.controller = controller;
    SmartDashboard.putNumber(TranslateConfig.kPKey, TranslateConfig.kP);
    SmartDashboard.putNumber(TranslateConfig.kIKey, TranslateConfig.kI);
    SmartDashboard.putNumber(TranslateConfig.kDKey, TranslateConfig.kD);
    forwardController =
        new PIDController(TranslateConfig.kP, TranslateConfig.kI, TranslateConfig.kD);

    addRequirements(vision, drive);

    forwardController.setIntegratorRange(TranslateConfig.minIntegral, TranslateConfig.maxIntegral);
  }

  @Override
  public void execute() {
    double forwardSpeed = 0.0;

    if (vision.getHasTarget()) {
      double range = vision.getDistToTarget();

      forwardSpeed = forwardController.calculate(range, 0);
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
    forwardController.setTolerance(TranslateConfig.kTolerance);
    return forwardController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(new Vector(0, 0), new Vector(0, 0), false, false);
  }
}
