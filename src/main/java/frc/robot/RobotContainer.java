// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Climb;
import frc.robot.constants.RobotConstants.OIConstants;
import frc.robot.subsystems.climber.Climber;

public class RobotContainer {
  private Joystick m_operatorController;
  private Climber m_climber;

  public RobotContainer() {
    m_operatorController = new Joystick(OIConstants.kOperatorJoystickPort);
    m_climber = new Climber();

    configureBindings();
  }

  private void configureBindings() {
    new Trigger(() -> m_operatorController.getRawButton(1))
      .whileTrue(new Climb(m_climber, false));

    new Trigger(() -> m_operatorController.getRawButton(2))
      .whileTrue(new Climb(m_climber, true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
