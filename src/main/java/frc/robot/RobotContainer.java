// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicDriveCommand;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.utils.Vector;

public class RobotContainer {
  private Drivetrain m_robotDrive;
  private Vision m_robotVision;

  // The driver's controller
  private XboxController m_driverController;
  private SendableChooser<Command> autoChooser;

  private Vector leftInputVec;
  private Vector rightInputVec;

  public RobotContainer() {
    m_robotVision = new Vision();
    m_robotDrive = new Drivetrain(m_robotVision);
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    registerCommands();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                    updateInput();
                    m_robotDrive.drive(
                          leftInputVec,
                          rightInputVec,
                          m_driverController.getRightBumper(),
                          m_driverController.getAButton());
                  },
            m_robotDrive));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void updateInput() {
    leftInputVec.setX(MathUtil.applyDeadband(
                              -m_driverController.getLeftY(), OIConstants.kDriveDeadband));
                leftInputVec.setY(MathUtil.applyDeadband(
                              -m_driverController.getLeftX(), OIConstants.kDriveDeadband));
                rightInputVec.setX(MathUtil.applyDeadband(
                              -m_driverController.getRightX(), OIConstants.kDriveDeadband));
                rightInputVec.setY(MathUtil.applyDeadband(
                              -m_driverController.getRightY(), OIConstants.kDriveDeadband));
  }

  // TODO: fill in placeholder commands with actual functionality
  private void registerCommands() {
    NamedCommands.registerCommand("intakeFromFloor", doNothing());
    NamedCommands.registerCommand("scoreAmp", doNothing());
    NamedCommands.registerCommand("aimAndScoreSpeaker", doNothing());
  }

  private Command doNothing() {
    return new Command() {};
  }

  private void configureBindings() {
    new Trigger(() -> triggerPressed())
        .whileTrue(new BasicDriveCommand(m_robotDrive, m_driverController));
  }

  public boolean triggerPressed() {
    if (m_driverController.getLeftTriggerAxis() != 0
        || m_driverController.getRightTriggerAxis() != 0) {
      return true;
    }
    return false;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
