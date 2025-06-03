package frc.robot.commands.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class StopShooter extends Command {
  private final Shooter m_shooter;

  public StopShooter(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }


  @Override
  public void initialize() {
    m_shooter.stopFlywheel();
  }

  @Override
  public void execute() {
  }

  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFlywheel();
  }
}
