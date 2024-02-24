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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicDriveCommand;
import frc.robot.commands.VisionTurnCommand;
import frc.robot.commands.shooter.*;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConstants.*;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.utils.Vector;

public class RobotContainer {
  private Joystick m_operatorController;
  private XboxController m_driverController;

  private POVButton m_manualAim;
  private POVButton m_autoAim;
  private POVButton m_speakerAim;

  private Vision m_vision;
  private Drivetrain m_robotDrive;
  private Shooter m_shooter;

  public RobotContainer() {
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);
    m_shooter = new Shooter();
    m_vision = new Vision();
    m_robotDrive = new Drivetrain(m_vision);
    configureBindings();

    m_shooter.setDefaultCommand(
        new RunCommand(() -> m_shooter.runFlywheel(ShooterConfig.kMaxFlywheelRPM), m_shooter));
  }

  //TODO 8 directional switch bindings
  private void configureBindings() {
    m_manualAim = new POVButton(m_operatorController, 180);
    m_autoAim = new POVButton(m_operatorController, 0);
    m_speakerAim = new POVButton(m_operatorController, 90);

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    new Vector(
                        MathUtil.applyDeadband(
                            -m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        MathUtil.applyDeadband(
                            -m_driverController.getLeftX(), OIConstants.kDriveDeadband)),
                    new Vector(
                        MathUtil.applyDeadband(
                            -m_driverController.getRightX(), OIConstants.kDriveDeadband),
                        MathUtil.applyDeadband(
                            -m_driverController.getRightY(), OIConstants.kDriveDeadband)),
                    m_driverController.getRightBumper(),
                    m_driverController.getAButton()),
            m_robotDrive));
    // just shoot on trigger
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShoot))
        .onTrue(new Shoot(m_shooter));
    // aim trap
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimTrap))
        .onTrue(new AimFromSetpoint(m_shooter, FieldElement.TRAP));
    // aim amp
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimAmp))
        .onTrue(new AimFromSetpoint(m_shooter, FieldElement.AMP));
    // stow shooter
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kStowShooter))
        .onTrue(new StowShooter(m_shooter));
    m_speakerAim.onTrue(new AimFromSetpoint(m_shooter, FieldElement.SPEAKER));
    m_manualAim.toggleOnTrue(
        new ManualAim(
            m_shooter, () -> m_operatorController.getRawAxis(Bindings.kManualAngleSlider)));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kToggleFlywheel))
        .toggleOnTrue(new ToggleFlywheel(m_shooter));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kRetractShield))
        .onTrue(new RetractShield(m_shooter));

    m_autoAim.toggleOnTrue(new Aim(m_shooter, m_vision));
    new Trigger(() -> triggerPressed())
        .whileTrue(new BasicDriveCommand(m_robotDrive, m_driverController));

    new Trigger(() -> m_driverController.getBButton())
        .onTrue(new VisionTurnCommand(m_vision, m_robotDrive, m_driverController));
  }

  public boolean triggerPressed() {
    if (m_driverController.getLeftTriggerAxis() != 0
        || m_driverController.getRightTriggerAxis() != 0) {
      return true;
    }
    return false;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
