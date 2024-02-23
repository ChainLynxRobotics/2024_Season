// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.*;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConstants.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  private Joystick m_operatorController;
  private Shooter m_shooter;
  private POVButton m_manualAim;
  private POVButton m_autoAim;
  private Vision m_vision;

  public RobotContainer() {
    m_operatorController = new Joystick(0);
    m_shooter = new Shooter();
    m_vision = new Vision();
    configureBindings();

    m_shooter.setDefaultCommand(
        new RunCommand(() -> m_shooter.runFlywheel(ShooterConfig.kMaxFlywheelRPM), m_shooter));
  }

  private void configureBindings() {
    // manual shoot
    /*new Trigger(() -> m_operatorController.getRawButton(5))
        .toggleOnTrue(
            new ManualAim(
                m_shooter, () -> m_operatorController.getRawAxis(Bindings.kManualAngleSlider)));*/
    // just shoot on trigger
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShoot))
        .onTrue(new Shoot(m_shooter));
    // aim speaker
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimSpeaker))
        .onTrue(new AimFromSetpoint(m_shooter, FieldElement.SPEAKER));
    // aim trap
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimTrap))
        .onTrue(new AimFromSetpoint(m_shooter, FieldElement.TRAP));
    // aim amp
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimAmp))
        .onTrue(new AimFromSetpoint(m_shooter, FieldElement.AMP));
    // stow shooter
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kStowShooter))
        .onTrue(new StowShooter(m_shooter));

    m_manualAim = new POVButton(m_operatorController, 180);
    m_manualAim
        .toggleOnTrue(
            new ManualAim(
                m_shooter, () -> m_operatorController.getRawAxis(Bindings.kManualAngleSlider)));
    m_autoAim = new POVButton(m_operatorController, 0);
    m_autoAim
        .toggleOnTrue(
            new Aim(
                m_shooter, m_vision));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
