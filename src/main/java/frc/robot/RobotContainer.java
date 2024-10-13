package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.climber.IndividualClimb;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.PivotMove;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.SpinFlywheels;
import frc.robot.commands.shooter.StowShooter;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConstants.Bindings;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {
  private Joystick m_operatorController;

  private POVButton m_trapAim;

  private Climber m_climber;
  private Shooter m_shooter;
  private Intake m_intake;
  private Drivetrain m_robotDrive;
  private Indexer m_indexer;

  // The driver's controller
  private XboxController m_driverController;

  @Log.NT(level = LogLevel.OVERRIDE_FILE_ONLY)
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_climber = new Climber();
    m_shooter = new Shooter();
    m_intake = new Intake();
    m_robotDrive = new Drivetrain();
    m_indexer = new Indexer();

    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new Joystick(OIConstants.kOperatorJoystickPort);

    registerCommands();
    // adds all autos in deploy dir to chooser
    autoChooser = new SendableChooser<Command>();
    configureBindings();

    autoChooser.addOption("LeaveFromTop", AutoBuilder.buildAuto("LeaveFromTop"));
    autoChooser.addOption("LeaveFromMid", AutoBuilder.buildAuto("LeaveFromMid"));
    autoChooser.addOption("LeaveFromBottom", AutoBuilder.buildAuto("LeaveFromBottom"));
    autoChooser.addOption("DoubleSpeaker", AutoBuilder.buildAuto("DoubleSpeaker"));
    autoChooser.addOption("TripleSpeaker", AutoBuilder.buildAuto("TripleSpeaker"));
    autoChooser.addOption("AmpThenSpeakerFromTop", AutoBuilder.buildAuto("AmpThenSpeakerFromTop"));
    autoChooser.addOption(
        "doAllTheThingsFromBottom", AutoBuilder.buildAuto("doAllTheThingsFromBottom"));
    autoChooser.addOption(
        "AmpThenSpeakerTwiceFromTop", AutoBuilder.buildAuto("AmpThenSpeakerTwiceFromTop"));

    autoChooser.addOption(
        "Shoot and leave straight from corner subwoofer",
        new SequentialCommandGroup(
            NamedCommands.getCommand("shootSpeaker"),
            new WaitCommand(2),
            new DriveStraight(
                m_robotDrive,
                1.0, // 66 inches per second at 150 degrees at 0.25 driveProp
                0.25,
                Units.degreesToRadians(150),
                true)));
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    Monologue.setupMonologue(this, "Robot", false, false);
  }

  private void configureBindings() {
    // angle on 8-directional button
    m_trapAim = new POVButton(m_operatorController, 90);
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              double xSpeed =
                  MathUtil.applyDeadband(
                      -m_driverController.getLeftY(), OIConstants.kDriveDeadband);
              double ySpeed =
                  MathUtil.applyDeadband(
                      -m_driverController.getLeftX(), OIConstants.kDriveDeadband);
              double rot =
                  MathUtil.applyDeadband(
                      -m_driverController.getRightX(), OIConstants.kDriveDeadband);

              SmartDashboard.putNumber("drive/controller left x", ySpeed);
              SmartDashboard.putNumber("drive/controller left y", xSpeed);
              SmartDashboard.putNumber("drive/controller right x", rot);

              m_robotDrive.drive(xSpeed, ySpeed, rot, true, true, m_driverController.getAButton());
            },
            m_robotDrive));

    // RunIntake constructor boolean is whether or not the intake should run reversed.
    new Trigger(this::getIntakeButton).whileTrue(new RunIntake(m_intake, m_indexer, false));
    new Trigger(this::getReverseIntakeButton).whileTrue(new RunIntake(m_intake, m_indexer, true));
    // just shoot on trigger
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShoot))
        .whileTrue(new Shoot(m_indexer, false));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShootReverse))
        .whileTrue(new Shoot(m_indexer, true));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kFlywheelAmp))
        .whileTrue(new SpinFlywheels(m_shooter, FieldElement.AMP));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kFlywheelSpeaker))
        .whileTrue(new SpinFlywheels(m_shooter, FieldElement.SPEAKER));

    m_trapAim.whileTrue(new SpinFlywheels(m_shooter, FieldElement.TRAP));

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

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kStowShooter))
        .whileTrue(new StowShooter(m_shooter));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimSpeaker))
        .whileTrue(new PivotMove(m_shooter, 0.3));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kAimAmp))
        .whileTrue(new PivotMove(m_shooter, 0.69));
  }

  public void periodic() {
    this.log("drivetrain/heading", m_robotDrive.getHeading());
    this.log("drivetrain/pose", m_robotDrive.getPose());

    this.log("climber/leftPos", m_climber.getLeft().getEncoder().getPosition());
    this.log("climber/rightPos", m_climber.getRight().getEncoder().getPosition());

    this.log("indexer/linebreak", m_indexer.getLineBreak());

    this.log(
        "shooter/angleRadians", m_shooter.getCurrentAngle().in(edu.wpi.first.units.Units.Radians));
    this.log("shooter/speedRPM", m_shooter.getCurrentRPM().baseUnitMagnitude());
    this.log(
        "shooter/flywheelController/P",
        m_shooter.getPidController().getP(),
        LogLevel.NOT_FILE_ONLY);
    this.log(
        "shooter/flywheelController/I",
        m_shooter.getPidController().getI(),
        LogLevel.NOT_FILE_ONLY);
    this.log(
        "shooter/flywheelController/D",
        m_shooter.getPidController().getD(),
        LogLevel.NOT_FILE_ONLY);
  }

  private void registerCommands() {
    // timeout doesn't need to be set because it is in a race group with the intake path in the
    // .path file
    NamedCommands.registerCommand("intakeFromFloor", new RunIntake(m_intake, m_indexer, false));

    NamedCommands.registerCommand(
        "shootSpeaker",
        new SequentialCommandGroup(
            new PivotMove(m_shooter, 0.3)
                .withTimeout(1), // Multiplier is 30% of max encoder rotations (160)
            new SpinFlywheels(m_shooter, FieldElement.SPEAKER).withTimeout(1.5),
            new ParallelCommandGroup(
                    new SpinFlywheels(m_shooter, FieldElement.SPEAKER), new Shoot(m_indexer, false))
                .withTimeout(3)));

    NamedCommands.registerCommand(
        "shootAmp",
        new SequentialCommandGroup(
            new PivotMove(m_shooter, 0.69).withTimeout(1),
            new SpinFlywheels(m_shooter, FieldElement.AMP).withTimeout(1.5),
            new ParallelCommandGroup(
                    new SpinFlywheels(m_shooter, FieldElement.AMP), new Shoot(m_indexer, false))
                .withTimeout(3)));
  }

  /**
   * Returns true if the intake is pressed; False otherwise.
   *
   * @see RobotConfig.IntakeConfig.Bindings.kIntakeNote
   */
  public boolean getIntakeButton() {
    return m_operatorController.getRawButton(Bindings.kIntakeNoteButtonID);
  }

  /**
   * Returns true if the reverse intake button is pressed; False otherwise.
   *
   * @see RobotConfig.IntakeConfig.Bindings.kReverseIntakeButtonID
   */
  public boolean getReverseIntakeButton() {
    return m_operatorController.getRawButton(Bindings.kReverseIntakeButtonID);
  }

  public boolean triggerPressed() {
    if (m_driverController.getLeftTriggerAxis() != 0
        || m_driverController.getRightTriggerAxis() != 0) {
      return true;
    }
    return false;
  }

  public void resetOdometry(Pose2d pose) {
    m_robotDrive.resetOdometry(pose);
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
