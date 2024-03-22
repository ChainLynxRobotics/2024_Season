package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeRollerMotor; // Intake roller motor

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    m_intakeRollerMotor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
  }

  /**
   * Runs the intake at given speed
   *
   * @param motorOutput Motor speed from -1.0 to 1.0 as a percentage
   */
  public void run(double motorOutput) {
    m_intakeRollerMotor.set(motorOutput);
  }

  /** Stops the roller motor */
  public void stop() {
    m_intakeRollerMotor.stopMotor();
  }
}
