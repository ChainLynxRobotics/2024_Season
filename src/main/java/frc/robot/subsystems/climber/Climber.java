package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class Climber extends SubsystemBase {

  private CANSparkMax leaderController;
  private CANSparkMax followerController;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;

  public Climber() {
    leaderController =
        new CANSparkMax(ClimberConstants.CLIMBER_CONTROLLER_ID1, MotorType.kBrushless);
    followerController =
        new CANSparkMax(ClimberConstants.CLIMBER_CONTROLLER_ID2, MotorType.kBrushless);

    followerController.follow(leaderController);

    m_pidController = leaderController.getPIDController();
    m_encoder = leaderController.getEncoder();

    // set PID coefficients
    m_pidController.setP(ClimberConstants.kClimberP);
    m_pidController.setI(ClimberConstants.kClimberI);
    m_pidController.setD(ClimberConstants.kClimberP);
    m_pidController.setIZone(ClimberConstants.kClimberIZone);
    m_pidController.setFF(ClimberConstants.kClimberFeedForward);
    m_pidController.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);

    SmartDashboard.putNumber("Setpoint", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setSetpoint();
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }

  public void setSetpoint() {
    double setpoint = SmartDashboard.getNumber("Setpoint", 0);
    m_pidController.setReference(metersToRotations(setpoint), CANSparkMax.ControlType.kPosition);
  }

  public double rotationsToMeters(double rotations) {
    return 2*Math.PI*ClimberConstants.kClimberMotorRadius*rotations;
  }

  public double metersToRotations(double meters) {
    return meters/(2*Math.PI*ClimberConstants.kClimberMotorRadius);
  }

  public void setMotorSpeed(double speed) {
    throw new UnsupportedOperationException("Unimplemented method 'setMotorSpeed'");
  }
}
