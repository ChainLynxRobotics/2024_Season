// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.IndividualClimb;
import frc.robot.constants.RobotConstants.Bindings;
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

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kRightClimberUp))
        .whileTrue(new IndividualClimb(m_climber, true, true));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kRightClimberDown))
        .whileTrue(new IndividualClimb(m_climber, true, false));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kLeftClimberUp))
        .whileTrue(new IndividualClimb(m_climber, false, true));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kLeftClimberDown))
        .whileTrue(new IndividualClimb(m_climber, false, false));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kBothClimbersUp))
        .whileTrue(new Climb(m_climber, true));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kBothClimbersDown))
        .whileTrue(new Climb(m_climber, false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
