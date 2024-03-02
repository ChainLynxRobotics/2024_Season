package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeRollerMotor; // Intake roller motor
  private final DigitalInput m_linebreak;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    m_intakeRollerMotor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
    // TODO maybe use to terminate intake command
    m_linebreak = new DigitalInput(IntakeConstants.kLineBreakSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake/linebreak sensor", m_linebreak.get());
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
