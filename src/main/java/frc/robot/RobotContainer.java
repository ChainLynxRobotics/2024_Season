// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.script.Bindings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicDriveCommand;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {
  private Drivetrain m_robotDrive;

  // The driver's controller
  XboxController m_driverController;
  
  public RobotContainer() {
    m_robotDrive = new Drivetrain();
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    configureBindings();

    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () ->
          m_robotDrive.drive(
            MathUtil.applyDeadband(
              -m_driverController.getLeftY(),
              OIConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              -m_driverController.getLeftX(),
              OIConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              -m_driverController.getRightX(),
              OIConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              -m_driverController.getRightY(),
              OIConstants.kDriveDeadband
            ),
            m_driverController.getRightBumper(),
            m_driverController.getAButton()
          ),
        m_robotDrive
      )
    );
  }

  private void configureBindings() {
    new Trigger(() -> triggerPressed())
      .whileTrue(new BasicDriveCommand(m_robotDrive, m_driverController));
  }

  public boolean triggerPressed() {
    if (
      m_driverController.getLeftTriggerAxis() != 0 ||
      m_driverController.getRightTriggerAxis() != 0
    ) {
      return true;
    }
    return false;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
