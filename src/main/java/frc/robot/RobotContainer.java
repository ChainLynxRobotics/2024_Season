// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.IndividualClimb;
import frc.robot.constants.RobotConstants.OIConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drivetrain;
import frc.utils.Vector;

public class RobotContainer {
  private Joystick m_operatorController;
  private XboxController m_driverController;

  private Climber m_climber;
  private Drivetrain m_robotDrive;
  private Vector leftInputVec;
  private Vector rightInputVec;

  public RobotContainer() {
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new Joystick(OIConstants.kOperatorJoystickPort);
    m_climber = new Climber();
    m_robotDrive = new Drivetrain();

    leftInputVec = new Vector();
    rightInputVec = new Vector();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              // update the values of leftInputVec and rightInputVec to the values of the controller
              // I'm avoiding re-instantiting Vectors to save memory
              updateInput();
              m_robotDrive.drive(
                  leftInputVec,
                  rightInputVec,
                  m_driverController.getRightBumper(),
                  m_driverController.getAButton());
            },
            m_robotDrive));

    configureBindings();
  }

  private void configureBindings() {
    // new Trigger(() -> m_operatorController.getRawButton(15)).whileTrue(new Climb(m_climber,
    // false));

    // new Trigger(() -> m_operatorController.getRawButton(16)).whileTrue(new Climb(m_climber,
    // true));

    new Trigger(() -> m_operatorController.getRawButton(11))
        .whileTrue(new IndividualClimb(m_climber, true, true));
    new Trigger(() -> m_operatorController.getRawButton(12))
        .whileTrue(new IndividualClimb(m_climber, true, false));
    new Trigger(() -> m_operatorController.getRawButton(13))
        .whileTrue(new IndividualClimb(m_climber, false, true));
    new Trigger(() -> m_operatorController.getRawButton(14))
        .whileTrue(new IndividualClimb(m_climber, false, false));

    // up
    new Trigger(() -> m_operatorController.getRawButton(15)).whileTrue(new Climb(m_climber, true));
    // down
    new Trigger(() -> m_operatorController.getRawButton(16)).whileTrue(new Climb(m_climber, false));
  }

  private void updateInput() {
    leftInputVec.setX(
        MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband));
    leftInputVec.setY(
        MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband));
    rightInputVec.setX(
        MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kDriveDeadband));
    rightInputVec.setY(
        MathUtil.applyDeadband(-m_driverController.getRightY(), OIConstants.kDriveDeadband));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
