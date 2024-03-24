package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
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
import frc.robot.commands.BasicDriveCommand;
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
import frc.utils.Vector;

public class RobotContainer {
  private Joystick m_operatorController;

  private POVButton m_autoAim;
  private POVButton m_trapAim;

  private Climber m_climber;
  private Shooter m_shooter;
  private Intake m_intake;
  private Drivetrain m_robotDrive;
  private Indexer m_indexer;

  // The driver's controller
  private XboxController m_driverController;
  private SendableChooser<Command> autoChooser;

  private Vector leftInputVec;
  private Vector rightInputVec;

  public RobotContainer() {
    m_climber = new Climber();
    m_shooter = new Shooter();
    m_intake = new Intake();
    m_robotDrive = new Drivetrain();
    m_indexer = new Indexer();

    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new Joystick(OIConstants.kOperatorJoystickPort);

    leftInputVec = new Vector();
    rightInputVec = new Vector();

    registerCommands();
    // adds all autos in deploy dir to chooser
    autoChooser = new SendableChooser<Command>();
    configureBindings();

    
    autoChooser.addOption(
        "Shoot and leave straight from corner subwoofer",
        new SequentialCommandGroup(
            NamedCommands.getCommand("shootSpeaker"),
            new WaitCommand(1),
            new DriveStraight(m_robotDrive, 1.5, 0.1, Math.PI / 2, true)));
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // angle on 8-directional button
    m_autoAim = new POVButton(m_operatorController, 0);
    m_trapAim = new POVButton(m_operatorController, 90);
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

    new Trigger(() -> triggerPressed())
        .whileTrue(new BasicDriveCommand(m_robotDrive, m_driverController));

    // RunIntake constructor boolean is whether or not the intake should run reversed.
    new Trigger(this::getIntakeButton).whileTrue(new RunIntake(m_intake, m_indexer, false));
    new Trigger(this::getReverseIntakeButton).whileTrue(new RunIntake(m_intake, m_indexer, true));
    // just shoot on trigger
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShoot))
        .whileTrue(new Shoot(m_indexer, false));
    new Trigger(() -> m_operatorController.getRawButton(Bindings.kShootReverse))
        .whileTrue(new Shoot(m_indexer, true));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kFlywheelAmp))
        .whileTrue(
            new SequentialCommandGroup(
                new SpinFlywheels(m_shooter, FieldElement.AMP).withTimeout(1.5),
                new ParallelCommandGroup(
                    new Shoot(m_indexer, false), new SpinFlywheels(m_shooter, FieldElement.AMP))));

    new Trigger(() -> m_operatorController.getRawButton(Bindings.kFlywheelSpeaker))
        .whileTrue(
            new SequentialCommandGroup(
                new SpinFlywheels(m_shooter, FieldElement.SPEAKER).withTimeout(1.5),
                new ParallelCommandGroup(
                    new Shoot(m_indexer, false),
                    new SpinFlywheels(m_shooter, FieldElement.SPEAKER))));

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

  // TODO: fill in placeholder commands with actual functionality
  private void registerCommands() {
    // timeout doesn't need to be set because it is in a race group with the intake path in the
    // .path file
    NamedCommands.registerCommand("intakeFromFloor", new RunIntake(m_intake, m_indexer, false));

    NamedCommands.registerCommand(
        "shootSpeaker",
        new SequentialCommandGroup(
            new PivotMove(m_shooter, 0.55).withTimeout(1),
            new SpinFlywheels(m_shooter, FieldElement.SPEAKER).withTimeout(1.5),
            new ParallelCommandGroup(
                    new SpinFlywheels(m_shooter, FieldElement.SPEAKER), new Shoot(m_indexer, false))
                .withTimeout(3)));

    NamedCommands.registerCommand(
        "shootAmp",
        new SequentialCommandGroup(
            new PivotMove(m_shooter, 0.3).withTimeout(1),
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
