/*
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.utils.Vector;

public class VisionTurnCommand extends Command {

  private Vision vision;
  private Drivetrain drive;
  private XboxController controller;

  private PIDController turnController;

  public VisionTurnCommand(Vision vision, Drivetrain drive, XboxController controller) {
    this.vision = vision;
    this.drive = drive;
    this.controller = controller;

    addRequirements(vision, drive);

    turnController = new PIDController(0.1, 0.2, 0.1);

    // set a limit on overshoot compensation
    turnController.setIntegratorRange(0, Math.toRadians(8));
  }

  @Override
  public void execute() {
    double rotationSpeed = -controller.getRightX();

    if (vision.getHasTarget()) {
      rotationSpeed = turnController.calculate(vision.getBestTarget().getYaw(), 0);
    } else {
      rotationSpeed = 0;
    }

    drive.drive(
        new Vector(
            MathUtil.applyDeadband(
                controller.getLeftX(), RobotConstants.DriveConstants.kDriveDeadband),
            MathUtil.applyDeadband(
                controller.getLeftY(), RobotConstants.DriveConstants.kDriveDeadband)),
        new Vector(
            MathUtil.applyDeadband(rotationSpeed, RobotConstants.DriveConstants.kDriveDeadband), 0),
        false,
        false);
  }

  @Override
  public void end(boolean interrupted) {
    drive.drive(new Vector(0, 0), new Vector(0, 0), false, false);
  }
}
*/