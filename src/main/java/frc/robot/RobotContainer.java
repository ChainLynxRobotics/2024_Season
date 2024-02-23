// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.*;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConstants.*;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
  private Joystick m_operatorController;
  private Shooter m_shooter;

  public RobotContainer() {
    m_operatorController = new Joystick(0);
    m_shooter = new Shooter();
    configureBindings();

    m_shooter.setDefaultCommand(
        new RunCommand(() -> m_shooter.runFlywheel(ShooterConfig.kMaxFlywheelRPM), m_shooter));
  }

  //TODO 8 directional switch bindings
  private void configureBindings() {
    // manual shoot
    new Trigger(() -> m_operatorController.getRawButton(5))
        .toggleOnTrue(
            new ManualAim(
                m_shooter, () -> m_operatorController.getRawAxis(Bindings.kManualAngleSlider)));
    // just shoot on trigger
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShoot))
        .onTrue(new Shoot(m_shooter));
    // aim speaker
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimSpeaker))
        .onTrue(new Aim(m_shooter, FieldElement.SPEAKER));
    // aim trap
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimTrap))
        .onTrue(new Aim(m_shooter, FieldElement.TRAP));
    // aim amp
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimAmp))
        .onTrue(new Aim(m_shooter, FieldElement.AMP));
    // stow shooter
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kStowShooter))
        .onTrue(new StowShooter(m_shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
