package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_rollerMotor; // right motor (Rollers)

  private final RelativeEncoder m_rollerEncoder; // Roller Relative Encoder

  // Constructs intake and initializes motor, PID, encoder objects, sensor
  public Intake() {
    m_rollerMotor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

    m_rollerEncoder = m_rollerMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake/roller velocity", m_rollerEncoder.getVelocity());
  }

  /**
   * Runs the intake
   *
   * @param motorOutput Motor speed from -1.0 to 1.0 as a percentage
   */
  public void run(double motorOutput) {
    m_rollerMotor.set(motorOutput);
  }

  // Stops the motor
  public void stop() {
    m_rollerMotor.stopMotor();
  }
}
