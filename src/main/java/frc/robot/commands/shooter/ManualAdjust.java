package frc.robot.commands.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.*;
import frc.robot.constants.RobotConfig.AdjustType;
=======
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.*;
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258
import frc.robot.subsystems.shooter.Shooter;

public class ManualAdjust extends Command {
  private final Shooter m_shooter;
  private final AdjustType m_type;
  private Measure<Angle> desiredAngle;
<<<<<<< HEAD
=======
  private Timer timer;
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258

  public ManualAdjust(Shooter shooter, AdjustType type) {
    m_shooter = shooter;
    m_type = type;
<<<<<<< HEAD
=======
    timer = new Timer();
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
<<<<<<< HEAD
=======
    timer.start();
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258
    switch (m_type) {
      case up:
        desiredAngle = m_shooter.getCurrentAngle().plus(ShooterConfig.kAdjustAmountDegrees);
        m_shooter.setAngle(desiredAngle);
        break;
      case down:
        desiredAngle = m_shooter.getCurrentAngle().minus(ShooterConfig.kAdjustAmountDegrees);
        m_shooter.setAngle(desiredAngle);
        break;
      default:
        desiredAngle = m_shooter.getCurrentAngle();
        m_shooter.setAngle(desiredAngle);
        break;
    }
  }

  @Override
<<<<<<< HEAD
  public boolean isFinished() {
    return m_shooter.isAtAngleSetpoint(desiredAngle.magnitude());
  }
}
=======
  public void execute() {
    m_shooter.setFF(Math.cos(m_shooter.getCurrentAngle().in(Units.Radians))*ShooterConfig.kAngleControlFF);
  }

  @Override
  public boolean isFinished() {
    return m_shooter.isAtAngleSetpoint(desiredAngle.magnitude()) || timer.get() > ShooterConfig.kAimTimeout;
  }
}
>>>>>>> b148e52d3bdbe5d24ad3569f4d855f3c25394258
