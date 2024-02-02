package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.units.Units.*;
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
  private CANSparkMax m_rollerMotorLeft;

  private CANSparkMax m_rollerMotorRight;

  private CANSparkMax m_angleMotorLeader;
  private CANSparkMax m_angleMotorFollower;

  private CANSparkMax m_hoodMotor;

  private SparkPIDController m_anglePidController;
  private SparkPIDController m_hoodPidController;

  private RelativeEncoder m_angleEncoder;
  private RelativeEncoder m_hoodEncoder;

  private boolean m_testModeCheck1; // Booleans for if test mode is enabled
  private boolean m_testModeCheck2;

  public Shooter() {
    m_rollerMotorLeft = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
    m_rollerMotorRight =
        new CANSparkMax(ShooterConstants.kRollerMotorRightId, MotorType.kBrushless);

    m_hoodMotor = new CANSparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushed);
    m_hoodPidController = m_hoodMotor.getPIDController();
    m_hoodEncoder = m_hoodMotor.getEncoder();

    m_angleMotorLeader =
        new CANSparkMax(ShooterConstants.kAngleMotorLeaderId, MotorType.kBrushless);
    m_angleMotorFollower =
        new CANSparkMax(ShooterConstants.kAngleMotorFollowerId, MotorType.kBrushless);
    m_angleMotorFollower.follow(m_angleMotorLeader);

    m_anglePidController = m_angleMotorLeader.getPIDController();
    m_angleEncoder = m_angleMotorLeader.getEncoder();

    // set Angle PID coefficients
    m_anglePidController.setP(RobotConfig.ShooterConfig.kAngleControlP);
    m_anglePidController.setI(RobotConfig.ShooterConfig.kAngleControlI);
    m_anglePidController.setD(RobotConfig.ShooterConfig.kAngleControlD);
    m_anglePidController.setFF(RobotConfig.ShooterConfig.kAngleControlFF);
    m_anglePidController.setIZone(RobotConfig.ShooterConfig.kAngleControlIZone);
    m_anglePidController.setOutputRange(
        RobotConfig.ShooterConfig.kAngleControlMinOutput,
        RobotConfig.ShooterConfig.kAngleControlMaxOutput);

    // set Hood PID coefficients
    m_hoodPidController.setP(RobotConfig.ShooterConfig.kHoodP);
    m_hoodPidController.setI(RobotConfig.ShooterConfig.kHoodI);
    m_hoodPidController.setD(RobotConfig.ShooterConfig.kHoodD);
    m_hoodPidController.setFF(RobotConfig.ShooterConfig.kHoodFF);
    m_hoodPidController.setIZone(RobotConfig.ShooterConfig.kHoodIZone);
    m_hoodPidController.setOutputRange(
        RobotConfig.ShooterConfig.kHoodMinOutput, RobotConfig.ShooterConfig.kHoodMaxOutput);

    putAngleOnSmartDashboard();
    putHoodOnSmartDashboard();

    // Both are required to enable test mode
    SmartDashboard.putBoolean(RobotConfig.ShooterConfig.kTestCheck1Key, m_testModeCheck1);
    SmartDashboard.putBoolean(RobotConfig.ShooterConfig.kTestCheck2Key, m_testModeCheck2);
  }

  public void putAngleOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard & test mode booleans
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

  public void putHoodOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard & test mode booleans
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodPGainKey, RobotConfig.ShooterConfig.kHoodP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodIGainKey, RobotConfig.ShooterConfig.kHoodI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodDGainKey, RobotConfig.ShooterConfig.kHoodD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodFFGainKey, RobotConfig.ShooterConfig.kHoodFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodIZoneKey, RobotConfig.ShooterConfig.kHoodIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodMinOutputKey,
        RobotConfig.ShooterConfig.kHoodMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kHoodMaxOutputKey,
        RobotConfig.ShooterConfig.kHoodMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    boolean testCheck1 = SmartDashboard.getBoolean(RobotConfig.ShooterConfig.kTestCheck1Key, false);
    boolean testCheck2 = SmartDashboard.getBoolean(RobotConfig.ShooterConfig.kTestCheck2Key, false);

    // Checks if test mode is enabled
    if (m_testModeCheck1 != testCheck1) {
      m_testModeCheck1 = testCheck1;
    }
    if (m_testModeCheck2 != testCheck2) {
      m_testModeCheck2 = testCheck2;
    }

    if (m_testModeCheck1 && m_testModeCheck2) {
      // read PID coefficients from SmartDashboard
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

      // read PID coefficients from SmartDashboard
      double pHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodPGainKey, 0);
      double iHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodIGainKey, 0);
      double dHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodDGainKey, 0);
      double izHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodIZoneKey, 0);
      double ffHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodFFGainKey, 0);
      double maxHood =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodMaxOutputKey, 0);
      double minHood =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kHoodMinOutputKey, 0);

      // checks PID values against Smartdash board
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

      // checks PID values against Smartdash board
      if (m_hoodPidController.getP() != pHood) {
        m_hoodPidController.setP(pHood);
      }
      if (m_hoodPidController.getI() != iHood) {
        m_hoodPidController.setI(iHood);
      }
      if (m_hoodPidController.getD() != dHood) {
        m_hoodPidController.setD(dHood);
      }
      if (m_hoodPidController.getFF() != ffHood) {
        m_hoodPidController.setFF(ffHood);
      }
      if (m_hoodPidController.getIZone() != izHood) {
        m_hoodPidController.setIZone(izHood);
      }
      if (m_hoodPidController.getOutputMax() != maxHood
          || m_hoodPidController.getOutputMin() != minHood) {
        m_hoodPidController.setOutputRange(minHood, maxHood);
      }
    }
  }

  // sets the target angle the shooter should be at
  public void setAngle(double targetAngleDegrees) {

  }

  // runs the rollers
  public void startFeedNote() {

  }

  // stops the rollers
  public void stopFeedNote() {

  }

  // runs the flywheel at a speed in rotations per minute
  public void runFlywheel(double targetRPM) {

  }

  // extends shield
  public void extendShield() {

  }

  // retracts shield
  public void retractShield() {

  }
  /*
  public boolean hasNote() {
    return Intake.isIndexed();
  }
  */
}
