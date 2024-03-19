package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig.ClimberConfig;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class Climber extends SubsystemBase {

  private CANSparkMax leaderController;
  private CANSparkMax followerController;
  private int multiplier;

  public Climber() {
    leaderController = new CANSparkMax(ClimberConstants.kClimberLeaderID, MotorType.kBrushless);
    followerController = new CANSparkMax(ClimberConstants.kClimberFollowerID, MotorType.kBrushless);

    leaderController.setIdleMode(IdleMode.kBrake);
    followerController.setIdleMode(IdleMode.kBrake);

    multiplier = 1;

    leaderController.getEncoder().setPosition(0);
    followerController.getEncoder().setPosition(0);

    SmartDashboard.putNumber("climber encoder rots", leaderController.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber encoder rots", leaderController.getEncoder().getPosition());
    if (leaderController.getEncoder().getPosition() < 0
        || leaderController.getEncoder().getPosition() > ClimberConfig.kUpperRotSoftStop) {
      leaderController.set(0);
      followerController.set(0);
    }
  }

  public double getLeaderEncoderPosition() {
    return leaderController.getEncoder().getPosition();
  }

  public void setBoth(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    leaderController.set(-0.5 * multiplier);
    followerController.set(0.5 * multiplier);
  }

  public void setLeader(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    leaderController.set(0.7 * multiplier);
  }

  public void stopLeader() {
    leaderController.set(0);
  }

  public void stopFollower() {
    followerController.set(0);
  }

  public void setFollower(boolean reverse) {
    multiplier = reverse ? -1 : 1;
    followerController.set(-0.7 * multiplier);
  }

  public CANSparkMax getLeader() {
    return leaderController;
  }

  public CANSparkMax getFollower() {
    return followerController;
  }
}
