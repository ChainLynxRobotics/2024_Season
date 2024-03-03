package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
// done
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class StowShooter extends Command {
  private final Shooter m_shooter;

  public StowShooter(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  @Override
<<<<<<< HEAD
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setAngle(Units.Degrees.of(ShooterConfig.kShooterStowAngle));
=======
  public void initialize() {
    m_shooter.setAngle(Units.Degrees.of(ShooterConfig.kShooterStowAngle));
  }

  @Override
  public void execute() {
    m_shooter.setFF(Math.cos(m_shooter.getCurrentAngle().in(Units.Radians))*ShooterConfig.kAngleControlFF);
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_shooter.isAtAngleSetpoint(ShooterConfig.kShooterStowAngle);
  }
<<<<<<< HEAD
}
=======
}
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258
