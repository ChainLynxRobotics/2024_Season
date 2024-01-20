package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommand extends Command {
  private final Climber m_subsystem;

  private double setPoint;

  /**
   * Creates a new ClimberCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(Climber subsystem, double setPoint) {
    m_subsystem = subsystem;
    this.setPoint = setPoint;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Set Rotations", setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  // If absolute value of error is less than or equal to tolerance, returns true
  @Override
  public boolean isFinished() {
    double error = SmartDashboard.getNumber("SetPoint", 0) - m_subsystem.getEncoderPosition();
    return Math.abs(error) <= RobotConstants.ClimberConstants.setPointTolerance;
  }

}
