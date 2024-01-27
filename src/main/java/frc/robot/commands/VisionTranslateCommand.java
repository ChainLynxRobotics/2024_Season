package frc.robot.Commands;

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
import org.photonvision.PhotonUtils;

public class VisionTranslateCommand extends Command {

    //private VisionSubsystem vision;
    private Vision vision;
    private Drivetrain drive;
    private XboxController controller;
    private double kp = 0.2;
    private double ki = 10;
    private double kd = 0.05;

    private PIDController forwardController;

    public VisionTranslateCommand(
        Vision vision,
        Drivetrain drive,
        XboxController controller
    ) {
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
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                RobotConstants.CAMERA_HEIGHT,
                RobotConstants.TARGET_HEIGHT,
                RobotConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(vision.getBestTarget().getPitch())
            );

            forwardSpeed =
                forwardController.calculate(range, RobotConstants.GOAL_RANGE);
        } else {
            forwardSpeed = 0;
        }

        drive.mainDrive(
            MathUtil.applyDeadband(forwardSpeed, DriveConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getLeftX(), DriveConstants.DRIVE_DEADBAND),
            MathUtil.applyDeadband(controller.getRightX(), DriveConstants.DRIVE_DEADBAND)
        );
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
        drive.drive(0, 0, 0, 0, false, false);
    }
}
