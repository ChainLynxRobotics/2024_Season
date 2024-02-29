package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax m_shooterRollerMotor;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    // Roller
    m_shooterRollerMotor =
        new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
