package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.constants.RobotConstants.ShooterConstants;

/**
 * Shoots notes through flywheels. Angle of shot controlled by pivot and notes fed into flywheels
 * from intake via indexer.
 */
public class Shooter extends SubsystemBase {
  /** 1. create motor and pid controller objects */
  private CANSparkMax m_rollerMotor;

  private CANSparkMax m_topFlywheelMotor;
  private CANSparkMax m_bottomFlywheelMotor;
  private RelativeEncoder m_topFlywheelEncoder;
  private RelativeEncoder m_bottomFlywheelEncoder;
  private SparkPIDController m_topFlywheelPIDController;

  private CANSparkMax m_shieldController;
  private RelativeEncoder m_shieldEncoder;

  private MutableMeasure<Velocity<Angle>> m_shooterSpeed;
  private MutableMeasure<Angle> m_shieldPosition;
  private MutableMeasure<Velocity<Distance>> m_targetVelocity;

  public Shooter() {
    // Roller
    m_rollerMotor = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);

    // Flywheel
    m_topFlywheelMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorId, MotorType.kBrushless);
    m_topFlywheelEncoder = m_topFlywheelMotor.getEncoder();
    m_topFlywheelPIDController = m_topFlywheelMotor.getPIDController();
    m_bottomFlywheelMotor =
        new CANSparkMax(ShooterConstants.kBottomFlywheelMotorId, MotorType.kBrushless);
    m_bottomFlywheelMotor.follow(m_topFlywheelMotor, true);
    m_bottomFlywheelEncoder = m_bottomFlywheelMotor.getEncoder();

    // shield
    m_shieldController = new CANSparkMax(ShooterConstants.kShieldMotorId, MotorType.kBrushless);
    m_shieldEncoder = m_shieldController.getEncoder();

    zeroEncoders();

    // set Flywheel PID coefficients
    m_topFlywheelPIDController.setP(RobotConfig.ShooterConfig.kTopFlywheelP);
    m_topFlywheelPIDController.setI(RobotConfig.ShooterConfig.kTopFlywheelI);
    m_topFlywheelPIDController.setD(RobotConfig.ShooterConfig.kTopFlywheelD);
    m_topFlywheelPIDController.setFF(RobotConfig.ShooterConfig.kTopFlywheelD);
    m_topFlywheelPIDController.setIZone(RobotConfig.ShooterConfig.kTopFlywheelIZone);
    m_topFlywheelPIDController.setOutputRange(
        RobotConfig.ShooterConfig.kTopFlywheelMinOutput,
        RobotConfig.ShooterConfig.kTopFlywheelMaxOutput);

    SmartDashboard.putNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());

    SmartDashboard.putNumber("Speaker Angle", ShooterConfig.kSpeakerAngle.magnitude());
    SmartDashboard.putNumber("Amp Angle", ShooterConfig.kAmpAngle.magnitude());
    SmartDashboard.putNumber("Trap Angle", ShooterConfig.kTrapAngle.magnitude());

    if (DriverStation.isTest()) {
      putAngleOnSmartDashboard();
    }
  }

  public void putAngleOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard

    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlPGainKey, RobotConfig.ShooterConfig.kAngleControlP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlIGainKey, RobotConfig.ShooterConfig.kAngleControlI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlDGainKey, RobotConfig.ShooterConfig.kAngleControlD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlFFGainKey,
        RobotConfig.ShooterConfig.kAngleControlFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlIZoneKey,
        RobotConfig.ShooterConfig.kAngleControlIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlMinOutputKey,
        RobotConfig.ShooterConfig.kAngleControlMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kAngleControlMaxOutputKey,
        RobotConfig.ShooterConfig.kAngleControlMaxOutput);
  }

  @Override
  public void periodic() {
    if (DriverStation.isTest()) {
      testPeriodic();
    }
  }

  void testPeriodic() {
    double flywheelRPM =
        SmartDashboard.getNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());

    if (m_topFlywheelEncoder.getVelocity() != flywheelRPM) {
      flywheelRPM = m_topFlywheelEncoder.getVelocity();
    }
  }

  public double degreesToRotations(double angle) {
    double rotation = angle / 360;
    return rotation;
  }

  public double rotationsToDegrees(double rotations) {
    double angle = rotations * 360;
    return angle;
  }

  // runs the rollers
  public void startFeedNote() {
    m_rollerMotor.set(RobotConfig.ShooterConfig.kRollerDefaultSpeed);
  }

  public void setShieldPosition(double position) {
    m_shieldController.getEncoder().setPosition(position);
  }

  // stops the rollers
  public void stopFeedNote() {
    m_rollerMotor.stopMotor();
  }

  // runs the flywheel at a speed in rotations per minute
  public void runFlywheel(double targetRPM) {
    System.out.println("running flywheel");
    m_topFlywheelPIDController.setReference(targetRPM, CANSparkBase.ControlType.kVelocity);
  }

  public void stopFlywheel() {
    m_topFlywheelMotor.stopMotor();
    ;
  }

  public void zeroEncoders() {
    m_topFlywheelEncoder.setPosition(0);
    m_bottomFlywheelEncoder.setPosition(0);
    m_shieldEncoder.setPosition(0);
  }

  public Measure<Velocity<Angle>> getCurrentRPM() {
    return m_shooterSpeed.mut_replace(m_topFlywheelEncoder.getVelocity(), Units.RPM);
  }

  public Measure<Velocity<Distance>> calculateVelocity(double targetY, Measure<Angle> targetAngle) {
    return m_targetVelocity.mut_replace(
        Math.sqrt(2 * ShooterConstants.Gravity * targetY)
            / (Math.sin(targetAngle.in(Units.Degrees))),
        Units.MetersPerSecond);
  }

  public double convertToRPM(double velocity) {
    // 0.0762 meters is diameter of flywheel
    double circumference = ShooterConstants.FlywheelDiameter * Math.PI;
    double rpm = velocity / circumference;
    return rpm;
  }

  // returns true if extended
  public boolean getShieldStatus() {
    return (Math.abs(m_shieldEncoder.getPosition()) > ShooterConfig.kShieldExtendedPosition);
  }

  public Measure<Angle> getShieldPosition() {
    return m_shieldPosition.mut_replace(m_shieldEncoder.getPosition(), Units.Rotations);
  }

  public void stopShieldMotor() {
    m_shieldController.stopMotor();
  }

  public boolean isAtFlywheelSetpoint(double setpoint) {
    return Math.abs(m_topFlywheelEncoder.getPosition() - setpoint)
        < ShooterConfig.kFlywheelError.magnitude();
  }
}
