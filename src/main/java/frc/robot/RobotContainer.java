// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class RobotContainer {
  private Joystick m_operatorController;
  private Shooter m_shooter;
  public RobotContainer() {
    m_operatorController = new Joystick(0);
    m_shooter = new Shooter();
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(() -> m_operatorController.getRawButton(2))
    .whileTrue(
      new ShootSpeakerManual(m_shooter, () -> m_operatorController.getRawButtonPressed(3), () -> m_operatorController.getRawAxis(0))
    );
    new Trigger(() -> m_operatorController.getRawButton(ShooterConfig.shootButton)).onTrue(new ShootSpeaker(m_shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
