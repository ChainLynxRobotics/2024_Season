package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class BasicDriveCommand extends Command {
  private Drivetrain m_drive;
  private XboxController m_controller;
  private double m_multiplier;

  public BasicDriveCommand(Drivetrain drive, XboxController controller) {
    this.m_drive = drive;
    this.m_controller = controller;

    this.m_multiplier = 1;

    addRequirements(this.m_drive);
  }

  @Override
  public void execute() {
    if (this.m_controller.getLeftTriggerAxis() != 0) {
      this.m_multiplier = 2;
      SmartDashboard.putBoolean("slow mode", false);
    } else if (this.m_controller.getRightTriggerAxis() != 0) {
      this.m_multiplier = 0.5;
      SmartDashboard.putBoolean("slow mode", true);
    }

    m_drive.drive(
        MathUtil.applyDeadband(
            -this.m_multiplier * this.m_controller.getLeftY(), DriveConstants.kDriveDeadband),
        MathUtil.applyDeadband(
            -this.m_multiplier * this.m_controller.getLeftX(), DriveConstants.kDriveDeadband),
        MathUtil.applyDeadband(
            -this.m_multiplier * this.m_controller.getRightX(), DriveConstants.kDriveDeadband),
        MathUtil.applyDeadband(
            -this.m_multiplier * this.m_controller.getRightY(), DriveConstants.kDriveDeadband),
        this.m_controller.getRightBumper(),
        this.m_controller.getAButton());
  }
}
