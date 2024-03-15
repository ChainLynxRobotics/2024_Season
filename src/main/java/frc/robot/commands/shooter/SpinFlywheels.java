package frc.robot.commands.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.FieldElement;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class SpinFlywheels extends Command {
  private final Shooter m_shooter;
  private final Vision m_vision;
  private final FieldElement m_type;
  private double desiredVelocity;
  private Measure<Angle> desiredAngle;

  public SpinFlywheels(Shooter shooter, FieldElement type) {
    m_shooter = shooter;
    m_vision = new Vision();
    m_type = type;

    addRequirements(m_shooter);
  }

  public SpinFlywheels(Shooter shooter, Vision eyes) {
    m_shooter = shooter;
    m_vision = eyes;
    m_type = null;
    addRequirements(m_shooter, m_vision);
  }

  @Override
  public void initialize() {
    if (m_type == null) {
      if (m_vision.getHasTarget()) {
        double desiredAngle =
            Units.Degrees.of(m_vision.getBestTarget().getPitch()).in(Units.Radians);
        Measure<Velocity<Distance>> desiredVelocity =
            m_shooter.calculateVelocity(
                m_vision.getDistToTarget() * Math.atan(desiredAngle),
                Units.Radians.of(desiredAngle));

        m_shooter.runFlywheel(m_shooter.convertToRPM(desiredVelocity.magnitude()));
      }
    } else {
      switch (m_type) {
        case AMP:
          desiredAngle = ShooterConfig.kAmpAngle;
          desiredVelocity = ShooterConfig.kDefaultAmpVelocity;
          break;
        case SPEAKER:
          desiredAngle = ShooterConfig.kSpeakerAngle;
          desiredVelocity = ShooterConfig.kDefaultSpeakerVelocity;
          break;
        case TRAP:
          desiredAngle = ShooterConfig.kTrapAngle;
          desiredVelocity = ShooterConfig.kDefaultTrapVelocity;
          break;
        default:
          desiredVelocity = 0;
          desiredAngle = Units.Degrees.of(0);
          desiredVelocity = 0;
          break;
      }
      m_shooter.runFlywheel(desiredVelocity);
    }
  }

  @Override
  public void execute() {
    double ff =
        Math.cos(m_shooter.getCurrentAngle().in(Units.Radians)) * ShooterConfig.kAngleControlFF;
    m_shooter.setFF(ff);
  }

  public boolean isFinished() {
    return m_shooter.isAtFlywheelSetpoint(desiredVelocity);
  }

  public double getVelocity(double elementHeight) {
    return m_shooter.convertToRPM(
        m_shooter.calculateVelocity(elementHeight, desiredAngle).magnitude());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopFlywheel();
  }
}
