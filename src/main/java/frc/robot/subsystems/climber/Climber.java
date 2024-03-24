package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class Climber extends SubsystemBase {

  private CANSparkMax leftController;
  private CANSparkMax rightController;
  private int multiplier;
  private DigitalInput m_limSwitchLeft;
  private DigitalInput m_limSwitchRight;

  public Climber() {
    leftController = new CANSparkMax(ClimberConstants.kClimberLeaderID, MotorType.kBrushless);
    rightController = new CANSparkMax(ClimberConstants.kClimberFollowerID, MotorType.kBrushless);

    leftController.setIdleMode(IdleMode.kBrake);
    rightController.setIdleMode(IdleMode.kBrake);

    m_limSwitchLeft = new DigitalInput(9);
    m_limSwitchRight = new DigitalInput(8);

    multiplier = 1;

    leftController.getEncoder().setPosition(0);
    rightController.getEncoder().setPosition(0);

    SmartDashboard.putNumber("climber encoder rots", leftController.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("left lim switch", m_limSwitchLeft.get());
    SmartDashboard.putBoolean("right lim switch", m_limSwitchRight.get());

    SmartDashboard.putNumber("climber encoder rots", leftController.getEncoder().getPosition());
    if (leftController.getEncoder().getPosition() < 0
        || leftController.getEncoder().getPosition() > ClimberConfig.kUpperRotSoftStop) {
      // leftController.set(0);
      // rightController.set(0);
    }

    /*if (!m_limSwitchLeft.get()) {
      leftController.set(0);
    }

    if (!m_limSwitchRight.get()) {
      rightController.set(0);
    }*/
  }

  public double getLeftEncoderPosition() {
    return leftController.getEncoder().getPosition();
  }

  public void setBoth(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    leftController.set(0.5 * multiplier);
    rightController.set(0.5 * multiplier);
  }

  public void setLeft(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    leftController.set(0.7 * multiplier);
  }

  public void stopLeft() {
    leftController.set(0);
  }

  public void stopRight() {
    rightController.set(0);
  }

  public void setRight(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    rightController.set(0.7 * multiplier);
  }

  public CANSparkMax getLeft() {
    return leftController;
  }

  public CANSparkMax getRight() {
    return rightController;
  }
}
