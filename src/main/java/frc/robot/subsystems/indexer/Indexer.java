package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class Indexer extends SubsystemBase {
  private CANSparkMax m_shooterRollerMotor;
  private DigitalInput m_lineBreakSensor;

  public Indexer() {
    // Roller
    m_shooterRollerMotor =
        new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
    m_lineBreakSensor = new DigitalInput(ShooterConstants.kLineBreakPort);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("line break sensor", m_lineBreakSensor.get());
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

  public boolean getLineBreak() {
    return m_lineBreakSensor.get();
  }
}
