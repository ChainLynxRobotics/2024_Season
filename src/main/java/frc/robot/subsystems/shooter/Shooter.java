package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
/**
 * two sets of rollers of the same size spin same direction,
 * index of seconday rollers make note go shooter, controller for flywheels,
 * difference of speed in flywheels determines angle and speed of note.
 * PID controller per motor to account for different dynamics (no follower)
 * periodic log velocity setpoints
 * pid only in subsystems
 * set net velocity, angle diff fywheel speed)
 *
**/
public class Shooter extends SubsystemBase {
  /**
   * 1. create motor and pid controller objects
   */
  private CANSparkMax m_rollerMotorLeft;
  private CANSparkMax m_rollerMotorRight;
  private CANSparkMax m_angleMotorLeader;
  private CANSparkMax m_angleMotorFollower;
  private DigitalInput m_linebreakSensor;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    m_rollerMotorLeft = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
    m_rollerMotorRight = new CANSparkMax(ShooterConstants.kRollerMotorRightId, MotorType.kBrushless);
    m_angleMotorLeader = new CANSparkMax(ShooterConstants.kAngleMotorLeaderId, MotorType.kBrushless);
    m_angleMotorFollower = new CANSparkMax(ShooterConstants.kAngleMotorFollowerId, MotorType.kBrushless);

    m_angleMotorFollower.follow(m_angleMotorLeader);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * runs the feed motors to feed the note to the shoooter flywheel and shoots the note
   *
   * @param speed the speed to run the feed motors from
   * -1 to 1 where -1 is full reverse and 1 is full forward
   */
  public void run(double speed) {

  }

  /**
   * runs the feed
   */
  public void runAmp() {
    run(1);
  }

  public boolean hasNote() {
    return m_linebreakSensor.get();
  }
}
