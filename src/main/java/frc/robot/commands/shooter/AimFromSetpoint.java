package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class AimFromSetpoint extends Command {
  private final Shooter m_shooter;
  private final FieldElement m_type;
  private double desiredAngle;

  public AimFromSetpoint(Shooter shooter, FieldElement type) {
    m_shooter = shooter;
    m_type = type;

    addRequirements(m_shooter);
  }

  //Sets shooter angle dependent on target.
  @Override
  public void initialize() {
    switch (m_type) {
      case SPEAKER:
        desiredAngle = ShooterConfig.kSpeakerAngle;
      case AMP:
        desiredAngle = ShooterConfig.kAmpAngle;
      case TRAP:
        desiredAngle = ShooterConfig.kTrapAngle;
      default:
        desiredAngle = 0;
    }
    if (m_type != FieldElement.SPEAKER) {
      m_shooter.extendShield();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setAngle(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopAngleMotor();
    m_shooter.stopShieldMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_shooter.getCurrentAngle() - desiredAngle) < ShooterConfig.kAngleError) {
      if(Math.abs(ShooterConfig.kShieldExtendedPosition - m_shooter.getShieldPosition()) <  ShooterConfig.kPositionError ||
       m_type != FieldElement.SPEAKER)
      return true;
    }
    return false;
  }
}
