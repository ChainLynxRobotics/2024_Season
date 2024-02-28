package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.intake.Intake;

public class RunIntakeReversed extends Command {
  private final Intake m_intake;

  /**
   * Creates a new RunIntake command, which runs the roller motor on the intake subsystem to intake
   * a note
   *
   * @param intake The subsystem used by this command.
   */
  public RunIntakeReversed(Intake intake) {
    m_intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Runs the intake at the default speed except reversed
    m_intake.run(RobotConfig.IntakeConfig.kDefaultSpeed * -1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
