package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
  private final Intake m_intake;

  /**
   * Creates a new RunIntake command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntake(Intake subsystem) {
    m_intake = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.run(RobotConfig.IntakeConfig.kDefaultSpeed);
  }

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
