package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
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

  private CANSparkMax m_topFlywheelMotor;
  private CANSparkMax m_bottomFlywheelMotor;

  private CANSparkMax m_shieldController;

  private SparkPIDController m_anglePidController;
  // separate pid controllers because we may need to spin them at different speeds
  private SparkPIDController m_topFlywheelPidController;
  private SparkPIDController m_shieldPidController;

  private RelativeEncoder m_angleEncoder;
  private RelativeEncoder m_topFlywheelEncoder;
  private RelativeEncoder m_bottomFlywheelEncoder;
  private RelativeEncoder m_shieldEncoder;

  private DigitalInput m_linebreakSensor;

  public Shooter() {
    m_rollerMotor = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);

    m_topFlywheelMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorId, MotorType.kBrushed);
    m_topFlywheelPidController = m_topFlywheelMotor.getPIDController();
    m_topFlywheelEncoder = m_topFlywheelMotor.getEncoder();
    m_bottomFlywheelMotor =
        new CANSparkMax(ShooterConstants.kBottomFlywheelMotorId, MotorType.kBrushed);
    m_bottomFlywheelMotor.follow(m_topFlywheelMotor);
    m_bottomFlywheelMotor.setInverted(true);
    m_bottomFlywheelEncoder = m_bottomFlywheelMotor.getEncoder();

    m_angleMotorLeader =
        new CANSparkMax(ShooterConstants.kAngleMotorLeaderId, MotorType.kBrushless);
    m_angleMotorFollower =
        new CANSparkMax(ShooterConstants.kAngleMotorFollowerId, MotorType.kBrushless);
    // sets follower motor to run inversely to the leader
    m_angleMotorFollower.follow(m_angleMotorLeader, true);

    m_shieldController = new CANSparkMax(ShooterConstants.kShieldMotorId, MotorType.kBrushless);
    m_shieldPidController = m_shieldController.getPIDController();
    m_shieldEncoder = m_shieldController.getEncoder();

    m_anglePidController = m_angleMotorLeader.getPIDController();
    m_angleEncoder = m_angleMotorLeader.getEncoder();

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

    // set top Flywheel PID coefficients
    m_topFlywheelPidController.setP(RobotConfig.ShooterConfig.kTopFlywheelP);
    m_topFlywheelPidController.setI(RobotConfig.ShooterConfig.kTopFlywheelI);
    m_topFlywheelPidController.setD(RobotConfig.ShooterConfig.kTopFlywheelD);
    m_topFlywheelPidController.setFF(RobotConfig.ShooterConfig.kTopFlywheelFF);
    m_topFlywheelPidController.setIZone(RobotConfig.ShooterConfig.kTopFlywheelIZone);
    m_topFlywheelPidController.setOutputRange(
        RobotConfig.ShooterConfig.kTopFlywheelMinOutput,
        RobotConfig.ShooterConfig.kTopFlywheelMaxOutput);

    // set Shield PID coefficients
    m_shieldPidController.setP(RobotConfig.ShooterConfig.kShieldP);
    m_shieldPidController.setI(RobotConfig.ShooterConfig.kShieldI);
    m_shieldPidController.setD(RobotConfig.ShooterConfig.kShieldD);
    m_shieldPidController.setFF(RobotConfig.ShooterConfig.kShieldFF);
    m_shieldPidController.setIZone(RobotConfig.ShooterConfig.kShieldIZone);
    m_shieldPidController.setOutputRange(
        RobotConfig.ShooterConfig.kShieldMinOutput, RobotConfig.ShooterConfig.kShieldMaxOutput);

    SmartDashboard.putNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Angle Degrees", m_angleEncoder.getPosition());

    putAngleOnSmartDashboard();
    putTopFlywheelOnSmartDashboard();
    putShieldOnSmartDashboard();
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

  public void putTopFlywheelOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelPGainKey, RobotConfig.ShooterConfig.kTopFlywheelP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelIGainKey, RobotConfig.ShooterConfig.kTopFlywheelI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelDGainKey, RobotConfig.ShooterConfig.kTopFlywheelD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelFFGainKey, RobotConfig.ShooterConfig.kTopFlywheelFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelIZoneKey,
        RobotConfig.ShooterConfig.kTopFlywheelIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelMinOutputKey,
        RobotConfig.ShooterConfig.kTopFlywheelMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelMaxOutputKey,
        RobotConfig.ShooterConfig.kTopFlywheelMaxOutput);
  }

  public void putShieldOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldPGainKey, RobotConfig.ShooterConfig.kShieldP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldIGainKey, RobotConfig.ShooterConfig.kShieldI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldDGainKey, RobotConfig.ShooterConfig.kShieldD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldFFGainKey, RobotConfig.ShooterConfig.kShieldFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldIZoneKey, RobotConfig.ShooterConfig.kShieldIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldMinOutputKey, RobotConfig.ShooterConfig.kShieldMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kShieldMaxOutputKey, RobotConfig.ShooterConfig.kShieldMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if note is indexed, spin flywheel to set velocity
    if (hasNote()) {
      runFlywheel(
          RobotConfig.ShooterConfig.kFlywheelDefaultRPM);
    }

    // if note is not indexed stop flywheel
    if (!hasNote()) {
      runFlywheel(0);
    }

    // read PID coefficients from SmartDashboard and stores them
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

    // read PID coefficients from SmartDashboard and stores them
    double pTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelPGainKey, 0);
    double iTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelIGainKey, 0);
    double dTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelDGainKey, 0);
    double izTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelIZoneKey, 0);
    double ffTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelFFGainKey, 0);
    double maxTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelMaxOutputKey, 0);
    double minTopFlywheel =
        SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelMinOutputKey, 0);


    // read PID coefficients from SmartDashboard and stores them
    double pShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldPGainKey, 0);
    double iShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldIGainKey, 0);
    double dShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldDGainKey, 0);
    double izShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldIZoneKey, 0);
    double ffShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldFFGainKey, 0);
    double maxShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldMaxOutputKey, 0);
    double minShield = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kShieldMinOutputKey, 0);

    double flywheelRPM = SmartDashboard.getNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());
    double angleDegrees = SmartDashboard.getNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());

    // checks PID values against Smartdash board and applies them to the PID if needed
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

    if (m_topFlywheelPidController.getP() != pTopFlywheel) {
      m_topFlywheelPidController.setP(pTopFlywheel);
    }
    if (m_topFlywheelPidController.getI() != iTopFlywheel) {
      m_topFlywheelPidController.setI(iTopFlywheel);
    }
    if (m_topFlywheelPidController.getD() != dTopFlywheel) {
      m_topFlywheelPidController.setD(dTopFlywheel);
    }
    if (m_topFlywheelPidController.getFF() != ffTopFlywheel) {
      m_topFlywheelPidController.setFF(ffTopFlywheel);
    }
    if (m_topFlywheelPidController.getIZone() != izTopFlywheel) {
      m_topFlywheelPidController.setIZone(izTopFlywheel);
    }
    if (m_topFlywheelPidController.getOutputMax() != maxTopFlywheel
        || m_topFlywheelPidController.getOutputMin() != minTopFlywheel) {
      m_topFlywheelPidController.setOutputRange(minTopFlywheel, maxTopFlywheel);
    }

    if (m_shieldPidController.getP() != pShield) {
      m_shieldPidController.setP(pShield);
    }
    if (m_shieldPidController.getI() != iShield) {
      m_shieldPidController.setI(iShield);
    }
    if (m_shieldPidController.getD() != dShield) {
      m_shieldPidController.setD(dShield);
    }
    if (m_shieldPidController.getFF() != ffShield) {
      m_shieldPidController.setFF(ffShield);
    }
    if (m_shieldPidController.getIZone() != izShield) {
      m_shieldPidController.setIZone(izShield);
    }
    if (m_shieldPidController.getOutputMax() != maxShield
        || m_shieldPidController.getOutputMin() != minShield) {
      m_shieldPidController.setOutputRange(minShield, maxShield);
    }

    if (m_topFlywheelEncoder.getVelocity() != flywheelRPM) {
      flywheelRPM = m_topFlywheelEncoder.getVelocity();
    }

    if (m_angleEncoder.getPosition() != degreeToRotations(angleDegrees)) {
      angleDegrees = rotationsToDegree(m_angleEncoder.getPosition());
    }

  }

  // sets the target angle the shooter should be at
  public void setAngle(double targetAngleDegrees) {
    m_anglePidController.setReference(degreeToRotations(targetAngleDegrees), CANSparkMax.ControlType.kPosition);
  }

  public void stopAngleMotor() {
    m_angleMotorLeader.set(0);
  }

  public double degreeToRotations(double angle) {
    double rotation = angle/360;
    return rotation;
  }

  public double rotationsToDegree(double rotations) {
    double angle = rotations * 360;
    return angle;
  }

  // runs the rollers
  public void startFeedNote() {
    m_rollerMotor.set(RobotConfig.ShooterConfig.kRollerDefaultSpeed);
  }

  // stops the rollers
  public void stopFeedNote() {
    m_rollerMotor.stopMotor();
  }

  // runs the flywheel at a speed in rotations per minute
  public void runFlywheel(double targetRPM) {
    m_topFlywheelPidController.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
  }

  // extends shield
  public void extendShield() {
    m_shieldPidController.setReference(
        RobotConfig.ShooterConfig.kShieldExtendedPosition, CANSparkMax.ControlType.kPosition);
  }

  // retracts shield
  public void retractShield() {
    m_shieldPidController.setReference(
        RobotConfig.ShooterConfig.kShieldRetractedPosition, CANSparkMax.ControlType.kPosition);
  }

  public boolean hasNote() {
    return m_linebreakSensor.get();
  }

  public void zeroEncoders() {
    m_angleEncoder.setPosition(0);
    m_topFlywheelEncoder.setPosition(0);
    m_bottomFlywheelEncoder.setPosition(0);
    m_shieldEncoder.setPosition(0);
  }

  public double getCurrentRPM() {
    double currentRPM = SmartDashboard.getNumber("Flywheel RPM", m_topFlywheelEncoder.getVelocity());
    return currentRPM;
  }

  public double getCurrentAngle() {
    double currentAngle = SmartDashboard.getNumber("Angle Degrees", degreeToRotations(m_angleEncoder.getPosition()));
    return currentAngle;
  }
}
