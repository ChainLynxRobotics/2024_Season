// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.*;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConstants.*;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  private Joystick m_operatorController;

  private POVButton m_autoAim;
  private POVButton m_speakerAim;

  private Vision m_vision;
  private Shooter m_shooter;

  public RobotContainer() {
    m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
    m_shooter = new Shooter();
    m_vision = new Vision();
    configureBindings();

    m_shooter.setDefaultCommand(
        new RunCommand(() -> m_shooter.runFlywheel(ShooterConfig.kDefaultFlywheelRPM), m_shooter));
  }

  private void configureBindings() {
    // angle on 8-directional button
    m_autoAim = new POVButton(m_operatorController, 0);
    m_speakerAim = new POVButton(m_operatorController, 90);

    // just shoot on trigger
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShoot))
        .onTrue(new Shoot(m_shooter));
    // aim amp (velocity only rn)
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimAmp))
        .onTrue(new Aim(m_shooter, FieldElement.AMP));

    m_speakerAim.onTrue(new Aim(m_shooter, FieldElement.SPEAKER));
    m_autoAim.whileTrue(new Aim(m_shooter, m_vision));

    // triggers for extending and retracting shield manually
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kRetractShield))
        .onTrue(new ActuateShield(m_shooter, false));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kExtendShield))
        .onTrue(new ActuateShield(m_shooter, true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
