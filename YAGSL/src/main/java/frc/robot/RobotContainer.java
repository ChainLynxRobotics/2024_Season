package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class RobotContainer {
  private static final double DEADBAND = 0.02;
  private SwerveDrive swerveDrive;
  private final CommandXboxController driverXbox = new CommandXboxController(0);


  public RobotContainer() {
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(14.5));
    } catch (IOException e) {
      System.out.println("Error opening file: " + e);
    }
  }

  public void defaultDriveCommand() {
    CommandScheduler.getInstance().schedule(new RunCommand(
      () -> swerveDrive.drive(
        new ChassisSpeeds(
          MathUtil.applyDeadband(-driverXbox.getLeftX(), DEADBAND),
          MathUtil.applyDeadband(-driverXbox.getLeftY(), DEADBAND),
          MathUtil.applyDeadband(-driverXbox.getRightX(), DEADBAND))
      )
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
