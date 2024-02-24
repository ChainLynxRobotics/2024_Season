package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.constants.RobotConstants.ShooterConstants;

/**
 * two sets of rollers of the same size spin same direction, index of seconday rollers make note go
 * shooter, controller for flywheels, difference of speed in flywheels determines angle and speed of
 * note. PID controller per motor to account for different dynamics (no follower) periodic log
 * velocity setpoints pid only in subsystems set net velocity, angle diff fywheel speed)
 */
public class Shooter extends SubsystemBase {
  /** 1. create motor and pid controller objects */
  private CANSparkMax m_rollerMotor;

  private CANSparkMax m_angleMotorLeader;
  private CANSparkMax m_angleMotorFollower;
  private SparkPIDController m_anglePidController;

  private RelativeEncoder m_angleEncoder;

  private CANSparkMax m_topFlywheelMotor;
  private CANSparkMax m_bottomFlywheelMotor;
  private RelativeEncoder m_topFlywheelEncoder;
  private RelativeEncoder m_bottomFlywheelEncoder;
  private SparkPIDController m_topFlywheelPIDController;

  private CANSparkMax m_shieldController;
  private RelativeEncoder m_shieldEncoder;

  private MutableMeasure<Velocity<Angle>> m_shooterSpeed;
  private MutableMeasure<Angle> m_shooterAngle;
  private MutableMeasure<Angle> m_targetAngle;
  private MutableMeasure<Angle> m_shieldPosition;

  public Shooter() {
    //Roller
    m_rollerMotor = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);

    //Flywheel
    m_topFlywheelMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorId, MotorType.kBrushed);
    m_topFlywheelEncoder = m_topFlywheelMotor.getEncoder();
    m_topFlywheelPIDController = m_topFlywheelMotor.getPIDController();
    m_bottomFlywheelMotor =
        new CANSparkMax(ShooterConstants.kBottomFlywheelMotorId, MotorType.kBrushed);
    m_bottomFlywheelMotor.follow(m_topFlywheelMotor, true);
    m_bottomFlywheelEncoder = m_bottomFlywheelMotor.getEncoder();

    //Angle
    m_angleMotorLeader =
        new CANSparkMax(ShooterConstants.kAngleMotorLeaderId, MotorType.kBrushless);
    m_angleMotorFollower =
        new CANSparkMax(ShooterConstants.kAngleMotorFollowerId, MotorType.kBrushless);
    // sets follower motor to run inversely to the leader
    m_angleMotorFollower.follow(m_angleMotorLeader, true);

    m_anglePidController = m_angleMotorLeader.getPIDController();
    m_angleEncoder = m_angleMotorLeader.getEncoder();

    //Shield
    m_shieldController = new CANSparkMax(ShooterConstants.kShieldMotorId, MotorType.kBrushless);
    m_shieldEncoder = m_shieldController.getEncoder();

    zeroEncoders();

    // set Angle PID coefficients
    m_anglePidController.setP(RobotConfig.ShooterConfig.kAngleControlP);
    m_anglePidController.setI(RobotConfig.ShooterConfig.kAngleControlI);
    m_anglePidController.setD(RobotConfig.ShooterConfig.kAngleControlD);
    m_anglePidController.setFF(RobotConfig.ShooterConfig.kAngleControlFF);
    m_anglePidController.setIZone(RobotConfig.ShooterConfig.kAngleControlIZone);
    m_anglePidController.setOutputRange(
        RobotConfig.ShooterConfig.kAngleControlMinOutput,
        RobotConfig.ShooterConfig.kAngleControlMaxOutput);

    //set Flywheel PID coefficients
    m_topFlywheelPIDController.setP(RobotConfig.ShooterConfig.kTopFlywheelP);
    m_topFlywheelPIDController.setI(RobotConfig.ShooterConfig.kTopFlywheelI);
    m_topFlywheelPIDController.setD(RobotConfig.ShooterConfig.kTopFlywheelD);
    m_topFlywheelPIDController.setFF(RobotConfig.ShooterConfig.kTopFlywheelD);
    m_topFlywheelPIDController.setIZone(RobotConfig.ShooterConfig.kTopFlywheelIZone);
    m_topFlywheelPIDController.setOutputRange(
        RobotConfig.ShooterConfig.kTopFlywheelMinOutput, RobotConfig.ShooterConfig.kTopFlywheelMaxOutput);

    SmartDashboard.putNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Angle Degrees", m_angleEncoder.getPosition());

    SmartDashboard.putNumber("Speaker Angle", ShooterConfig.kSpeakerAngle);
    SmartDashboard.putNumber("Amp Angle", ShooterConfig.kAmpAngle);
    SmartDashboard.putNumber("Trap Angle", ShooterConfig.kTrapAngle);

    if(DriverStation.isTest())
    {
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
    // This method will be called once per scheduler run
    if(DriverStation.isTest())
    {
      testPeriodic();
    }
  }

  void testPeriodic()
  {
    double pAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlPGainKey, 0);
    double iAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlIGainKey, 0);
    double dAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlDGainKey, 0);
    double izAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlIZoneKey, 0);
    double ffAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlFFGainKey, 0);
    double maxAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlMaxOutputKey, 0);
    double minAngleController =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kAngleControlMinOutputKey, 0);

    double flywheelRPM =
        SmartDashboard.getNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());
    double angleDegrees =
        SmartDashboard.getNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());

    double speakerAngle = SmartDashboard.getNumber("Speaker Angle", ShooterConfig.kSpeakerAngle);
    double ampAngle = SmartDashboard.getNumber("Amp Angle", ShooterConfig.kAmpAngle);
    double trapAngle = SmartDashboard.getNumber("Trap Angle", ShooterConfig.kTrapAngle);

    // checks PID values against Smart dashboard and applies them to the PID if needed
    if (m_anglePidController.getP() != pAngleController) {
      m_anglePidController.setP(pAngleController);
    }
    if (m_anglePidController.getI() != iAngleController) {
      m_anglePidController.setI(iAngleController);
    }
    if (m_anglePidController.getD() != dAngleController) {
      m_anglePidController.setD(dAngleController);
    }
    if (m_anglePidController.getFF() != ffAngleController) {
      m_anglePidController.setFF(ffAngleController);
    }
    if (m_anglePidController.getIZone() != izAngleController) {
      m_anglePidController.setIZone(izAngleController);
    }
    if (m_anglePidController.getOutputMax() != maxAngleController
        || m_anglePidController.getOutputMin() != minAngleController) {
      m_anglePidController.setOutputRange(minAngleController, maxAngleController);
    }

    if (m_topFlywheelEncoder.getVelocity() != flywheelRPM) {
      flywheelRPM = m_topFlywheelEncoder.getVelocity();
    }

    if (m_angleEncoder.getPosition() != degreesToRotations(angleDegrees)) {
      angleDegrees = rotationsToDegrees(m_angleEncoder.getPosition());
    }

    if (ShooterConfig.kSpeakerAngle != speakerAngle) {
      ShooterConfig.kSpeakerAngle = speakerAngle;
    }
    if (ShooterConfig.kAmpAngle != ampAngle) {
      ShooterConfig.kAmpAngle = ampAngle;
    }
    if (ShooterConfig.kTrapAngle != trapAngle) {
      ShooterConfig.kTrapAngle = trapAngle;
    }
  }

  // sets the target angle the shooter should be at
  public void setAngle(Measure<Angle> targetAngle) {
    // TODO
    m_anglePidController.setReference(degreesToRotations(targetAngle) - 30);
  }

  public void stopAngleMotor() {
    m_angleMotorLeader.stopMotor();
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
    m_topFlywheelPIDController.setReference(targetRPM, CANSparkBase.ControlType.kVelocity);
  }

  public void stopFlywheel() {
    m_topFlywheelMotor.stopMotor();;
  }

  public void zeroEncoders() {
    m_angleEncoder.setPosition(0);
    m_topFlywheelEncoder.setPosition(0);
    m_bottomFlywheelEncoder.setPosition(0);
    m_shieldEncoder.setPosition(0);
  }

  public Measure<Velocity<Angle>> getCurrentRPM() {
    return m_shooterSpeed.mut_replace(m_topFlywheelEncoder.getVelocity(), Units.RPM);
  }

  public Measure<Angle> getCurrentAngle() {
    return m_shooterAngle.mut_replace(m_angleEncoder.getPosition(), Units.Revolutions);
  }

  public Measure<Angle> calculateAngle(double targetX, double targetY) {
    return m_targetAngle.mut_replace(Math.atan2(targetY, targetX), Units.Degrees);
  }

  /*public Measure<Velocity> calculateVelocity(double targetX, double targetY) {
    
    return m_targetVelocity.mut_replace();
  }*/

  public double convertToRPM(double velocity) {
    // 0.0762 is diameter of flywheel
    double circumference = ShooterConstants.FlywheelDiameter * Math.PI;
    double rpm = velocity / circumference;
    return rpm;
  }

  public boolean isShieldExtended() {
    return (Math.abs(getShieldPosition().in(Units.Rotations)) > ShooterConstants.kShieldExtentionAngle.in(Units.Rotations));
  }

  public Measure<Angle> getShieldPosition() {
    return m_shieldPosition.mut_replace(m_shieldEncoder.getPosition(), Units.Rotations);
  }

  public void stopShieldMotor() {
    m_shieldController.stopMotor();
  }
}
