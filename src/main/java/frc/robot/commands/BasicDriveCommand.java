package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class BasicDriveCommand extends Command {
    private Drivetrain drive;
    private XboxController controller;
    private double multiplier;

    public BasicDriveCommand(Drivetrain drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;

        multiplier = 1;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        boolean slowMode = false;
        if (controller.getLeftTriggerAxis() != 0) {
            multiplier = 2;
            slowMode = false;
        } else if (controller.getRightTriggerAxis() != 0) {
            multiplier = 0.5;
            slowMode = true;
        }

        SmartDashboard.putBoolean("slow mode", slowMode);


        drive.drive(
            MathUtil.applyDeadband(
                -multiplier * controller.getLeftY(),
                DriveConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
                -multiplier * controller.getLeftX(),
                DriveConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
                -multiplier * controller.getRightX(),
                DriveConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
                -multiplier * controller.getRightY(),
                DriveConstants.kDriveDeadband
            ),
            controller.getRightBumper(),
            controller.getAButton()
        );
    }
}
