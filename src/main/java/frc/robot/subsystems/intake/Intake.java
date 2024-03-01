package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax m_shooterRollerMotor;
  private final CANSparkMax m_intakeRollerMotor; // Intake roller motor
  private final DigitalInput m_linebreak;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    // Roller
    m_shooterRollerMotor =
        new CANSparkMax(ShooterConstants.kRollerMotorId, MotorType.kBrushless);
    m_intakeRollerMotor = new CANSparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
    // TODO maybe use to terminate intake command
    m_linebreak = new DigitalInput(IntakeConstants.kLineBreakSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake/linebreak sensor", m_linebreak.get());
  }

  // runs the rollers
  public void startFeedNote(boolean reverse) {
    if (reverse) {
      m_shooterRollerMotor.set(-RobotConfig.ShooterConfig.kRollerDefaultSpeed);
    } else {
      m_shooterRollerMotor.set(RobotConfig.ShooterConfig.kRollerDefaultSpeed);
    }
  }

  // stops the rollers
  public void stopFeedNote() {
    m_shooterRollerMotor.stopMotor();
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
