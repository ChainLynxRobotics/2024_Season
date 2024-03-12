package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ClimberConstants;
import java.util.ArrayList;
import java.util.List;

public class Climber extends SubsystemBase {

  private CANSparkMax leaderController;
  private CANSparkMax followerController;
  private SparkPIDController m_leaderPidController;
  private SparkPIDController m_followerPidController;
  private List<SparkPIDController> m_pidControllers;

  public Climber() {
    leaderController = new CANSparkMax(ClimberConstants.kClimberLeaderID, MotorType.kBrushless);
    followerController = new CANSparkMax(ClimberConstants.kClimberFollowerID, MotorType.kBrushless);

    leaderController.setIdleMode(IdleMode.kBrake);
    followerController.setIdleMode(IdleMode.kBrake);

    followerController.follow(leaderController);

    m_leaderPidController = leaderController.getPIDController();
    m_followerPidController = followerController.getPIDController();
    m_pidControllers = new ArrayList<>();
    m_pidControllers.add(m_leaderPidController);
    m_pidControllers.add(m_followerPidController);
    leaderController.getEncoder().setPosition(0);

    // set PID coefficients
    m_pidControllers.forEach(
        (m_pidController) -> {
          m_pidController.setP(ClimberConstants.kClimberP);
          m_pidController.setI(ClimberConstants.kClimberI);
          m_pidController.setD(ClimberConstants.kClimberP);
          m_pidController.setIZone(ClimberConstants.kClimberIZone);
          m_pidController.setFF(ClimberConstants.kClimberFeedForward);
          m_pidController.setOutputRange(
              ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);
        });

    SmartDashboard.putNumber("climber encoder rots", leaderController.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber encoder rots", leaderController.getEncoder().getPosition());
  }

  public double getLeaderEncoderPosition() {
    return leaderController.getEncoder().getPosition();
  }

  public double getFollowerEncoderPosition() {
    return followerController.getEncoder().getPosition();
  }

  public void setSetpoint(SparkPIDController m_pidController, double setpoint) {
    m_pidController.setReference(metersToRotations(setpoint), CANSparkMax.ControlType.kPosition);
  }

  public static double rotationsToMeters(double rotations) {
    return 2 * Math.PI * ClimberConstants.kClimberMotorRadius * rotations;
  }

  // TODO determine climber rot conversion factor empirically
  public static double metersToRotations(double meters) {
    return meters / (2 * Math.PI * ClimberConstants.kClimberMotorRadius);
  }

  public void setMotorSpeed(double speed) {
    leaderController.set(speed);
  }

  public void resetFollower() {
    followerController.follow((leaderController));
  }

  public SparkPIDController getLeaderPidController() {
    return m_leaderPidController;
  }

  public SparkPIDController getFollowerPidController() {
    return m_followerPidController;
  }
}
