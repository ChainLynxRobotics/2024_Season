package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_rollerFollowerMotor; // left motor (Rollers)
  private final CANSparkMax m_rollerLeaderMotor; // right motor (Rollers)

  private final RelativeEncoder m_rollerEncoder; // Roller Relative Encoder

  private final DigitalInput m_intakeSensor; // Line break note sensor

  // Constructs intake and initializes motor, PID, encoder objects, sensor
  public Intake() {
    m_rollerFollowerMotor = new CANSparkMax(IntakeConstants.kFollowerMotorID, MotorType.kBrushless);
    m_rollerLeaderMotor = new CANSparkMax(IntakeConstants.kLeaderMotorID, MotorType.kBrushless);
    m_rollerFollowerMotor.follow(this.m_rollerLeaderMotor);

    m_rollerEncoder = m_rollerLeaderMotor.getEncoder();
    zeroEncoders();

    m_intakeSensor = new DigitalInput(RobotConstants.IntakeConstants.kLineBreakSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Runs the intake
   *
   * @param motorOutput Motor speed from -1.0 to 1.0 as a percentage
   */
  public void run(double motorOutput) {
    m_rollerLeaderMotor.set(motorOutput);
  }

  /**
   * Runs the intake at a default speed The motor can be reversed by inputing a true as its
   * parameter
   *
   * @param isReversed The input to reverse the motor speed
   */
  public void run(boolean isReversed) {
    double runSpeed =
        isReversed
            ? -RobotConfig.IntakeConfig.kDefaultSpeed
            : RobotConfig.IntakeConfig.kDefaultSpeed;
    run(runSpeed);
  }

  // Checks if note is indexed
  public boolean isIndexed() {
    return m_intakeSensor.get();
  }

  // Stops the motor
  public void stop() {
    m_rollerLeaderMotor.stopMotor();
  }

  // Zeros the encoder(s)
  public void zeroEncoders() {
    m_rollerEncoder.setPosition(0);
  }
}
