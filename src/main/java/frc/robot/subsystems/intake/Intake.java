package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_angleMotor; // intake angle motor

  private final CANSparkMax m_rollerFollowerMotor; // left motor
  private final CANSparkMax m_rollerLeaderMotor; // right motor

  // Constructs intake, and initializes motor objects
  public Intake() {
    this.m_angleMotor = new CANSparkMax(IntakeConstants.kAngleMotorID, MotorType.kBrushless);

    this.m_rollerFollowerMotor = new CANSparkMax(
      IntakeConstants.kFollowerMotorID, MotorType.kBrushless
    );
    this.m_rollerLeaderMotor = new CANSparkMax(
      IntakeConstants.kLeaderMotorID, MotorType.kBrushless
    );
    this.m_rollerFollowerMotor.follow(this.m_rollerLeaderMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Runs the intake
   *
   * @param motorOutput Motor speed from -1.0 to 1.0 as a percentage
   */
  public void run(double motorOutput) {
    this.m_rollerLeaderMotor.set(motorOutput);
  }

  /**
   * This code runs the intake at a default speed The motor can be reversed by inputing a true as
   * its parameter
   *
   * @param isReversed The input to reverse the motor speed
   */
  public void run(boolean isReversed) {
    if (isReversed) {
      run(-IntakeConstants.kDefaultSpeed);
    } else {
      run(-IntakeConstants.kDefaultSpeed);
    }
  }

  // Stops the motor
  public void stop() {
    this.m_rollerLeaderMotor.stopMotor();
  }
}
