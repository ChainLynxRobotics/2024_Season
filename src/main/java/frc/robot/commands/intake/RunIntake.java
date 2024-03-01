package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
  private final Intake m_intake;
  private boolean m_reversed;

  /**
   * Creates a new RunIntake command, which runs the roller motor on the intake subsystem to intake
   * a note
   *
   * @param intake The subsystem used by this command.
   */
  public RunIntake(Intake intake, boolean reversed) {
    m_intake = intake;
    m_reversed = reversed;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int multiplier = m_reversed ? -1 : 1;
    m_intake.run(RobotConfig.IntakeConfig.kDefaultSpeed * multiplier);
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