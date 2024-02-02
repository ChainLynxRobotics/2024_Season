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

  private CANSparkMax m_topHoodMotor;

  private SparkPIDController m_anglePidController;
  private SparkPIDController m_topHoodPidController;

  private RelativeEncoder m_angleEncoder;
  private RelativeEncoder m_topHoodEncoder;

  private boolean m_testModeCheck1; // Booleans for if test mode is enabled
  private boolean m_testModeCheck2;

  public Shooter() {
    m_rollerMotorLeft = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
    m_rollerMotorRight =
        new CANSparkMax(ShooterConstants.kRollerMotorRightId, MotorType.kBrushless);

    m_topHoodMotor = new CANSparkMax(ShooterConstants.kTopHoodMotorId, MotorType.kBrushed);
    m_topHoodPidController = m_topHoodMotor.getPIDController();
    m_topHoodEncoder = m_topHoodMotor.getEncoder();

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

    // set top Hood PID coefficients
    m_topHoodPidController.setP(RobotConfig.ShooterConfig.kTopHoodP);
    m_topHoodPidController.setI(RobotConfig.ShooterConfig.kTopHoodI);
    m_topHoodPidController.setD(RobotConfig.ShooterConfig.kTopHoodD);
    m_topHoodPidController.setFF(RobotConfig.ShooterConfig.kTopHoodFF);
    m_topHoodPidController.setIZone(RobotConfig.ShooterConfig.kTopHoodIZone);
    m_topHoodPidController.setOutputRange(
        RobotConfig.ShooterConfig.kTopHoodMinOutput, RobotConfig.ShooterConfig.kTopHoodMaxOutput);

    putAngleOnSmartDashboard();
    putTopHoodOnSmartDashboard();

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

  public void putTopHoodOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard & test mode booleans
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodPGainKey, RobotConfig.ShooterConfig.kTopHoodP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodIGainKey, RobotConfig.ShooterConfig.kTopHoodI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodDGainKey, RobotConfig.ShooterConfig.kTopHoodD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodFFGainKey, RobotConfig.ShooterConfig.kTopHoodFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodIZoneKey, RobotConfig.ShooterConfig.kTopHoodIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodMinOutputKey,
        RobotConfig.ShooterConfig.kTopHoodMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopHoodMaxOutputKey,
        RobotConfig.ShooterConfig.kTopHoodMaxOutput);
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
      double pTopHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodPGainKey, 0);
      double iTopHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodIGainKey, 0);
      double dTopHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodDGainKey, 0);
      double izTopHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodIZoneKey, 0);
      double ffTopHood = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodFFGainKey, 0);
      double maxTopHood =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodMaxOutputKey, 0);
      double minTopHood =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopHoodMinOutputKey, 0);

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
      if (m_topHoodPidController.getP() != pTopHood) {
        m_topHoodPidController.setP(pTopHood);
      }
      if (m_topHoodPidController.getI() != iTopHood) {
        m_topHoodPidController.setI(iTopHood);
      }
      if (m_topHoodPidController.getD() != dTopHood) {
        m_topHoodPidController.setD(dTopHood);
      }
      if (m_topHoodPidController.getFF() != ffTopHood) {
        m_topHoodPidController.setFF(ffTopHood);
      }
      if (m_topHoodPidController.getIZone() != izTopHood) {
        m_topHoodPidController.setIZone(izTopHood);
      }
      if (m_topHoodPidController.getOutputMax() != maxTopHood
          || m_topHoodPidController.getOutputMin() != minTopHood) {
        m_topHoodPidController.setOutputRange(minTopHood, maxTopHood);
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
