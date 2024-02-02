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

  private CANSparkMax m_topFlywheelMotor;
  private CANSparkMax m_bottomFlywheelMotor;

  private SparkPIDController m_anglePidController;
  // separate pid controllers because we may need to spin them at different speeds
  private SparkPIDController m_topFlywheelPidController;
  private SparkPIDController m_bottomFlywheelPidController;

  private RelativeEncoder m_angleEncoder;
  private RelativeEncoder m_topFlywheelEncoder;
  private RelativeEncoder m_bottomFlywheelEncoder;

  private boolean m_testModeCheck1; // Booleans for if test mode is enabled
  private boolean m_testModeCheck2;

  public Shooter() {
    m_rollerMotorLeft = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
    m_rollerMotorRight =
        new CANSparkMax(ShooterConstants.kRollerMotorRightId, MotorType.kBrushless);

    m_topFlywheelMotor = new CANSparkMax(ShooterConstants.kTopFlywheelMotorId, MotorType.kBrushed);
    m_topFlywheelPidController = m_topFlywheelMotor.getPIDController();
    m_topFlywheelEncoder = m_topFlywheelMotor.getEncoder();
    m_bottomFlywheelMotor = new CANSparkMax(ShooterConstants.kBottomFlywheelMotorId, MotorType.kBrushed);
    m_bottomFlywheelPidController = m_bottomFlywheelMotor.getPIDController();
    m_bottomFlywheelEncoder = m_bottomFlywheelMotor.getEncoder();

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

    // set top Flywheel PID coefficients
    m_topFlywheelPidController.setP(RobotConfig.ShooterConfig.kTopFlywheelP);
    m_topFlywheelPidController.setI(RobotConfig.ShooterConfig.kTopFlywheelI);
    m_topFlywheelPidController.setD(RobotConfig.ShooterConfig.kTopFlywheelD);
    m_topFlywheelPidController.setFF(RobotConfig.ShooterConfig.kTopFlywheelFF);
    m_topFlywheelPidController.setIZone(RobotConfig.ShooterConfig.kTopFlywheelIZone);
    m_topFlywheelPidController.setOutputRange(
        RobotConfig.ShooterConfig.kTopFlywheelMinOutput, RobotConfig.ShooterConfig.kTopFlywheelMaxOutput);

    // set bottom Flywheel PID coefficients
    m_bottomFlywheelPidController.setP(RobotConfig.ShooterConfig.kBottomFlywheelP);
    m_bottomFlywheelPidController.setI(RobotConfig.ShooterConfig.kBottomFlywheelI);
    m_topFlywheelPidController.setD(RobotConfig.ShooterConfig.kBottomFlywheelD);
    m_bottomFlywheelPidController.setFF(RobotConfig.ShooterConfig.kBottomFlywheelFF);
    m_bottomFlywheelPidController.setIZone(RobotConfig.ShooterConfig.kBottomFlywheelIZone);
    m_bottomFlywheelPidController.setOutputRange(
        RobotConfig.ShooterConfig.kBottomFlywheelMinOutput, RobotConfig.ShooterConfig.kBottomFlywheelMaxOutput);

    putAngleOnSmartDashboard();
    putTopFlywheelOnSmartDashboard();
    putBottomFlywheelOnSmartDashboard();

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

  public void putTopFlywheelOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard & test mode booleans
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelPGainKey, RobotConfig.ShooterConfig.kTopFlywheelP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelIGainKey, RobotConfig.ShooterConfig.kTopFlywheelI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelDGainKey, RobotConfig.ShooterConfig.kTopFlywheelD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelFFGainKey, RobotConfig.ShooterConfig.kTopFlywheelFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelIZoneKey, RobotConfig.ShooterConfig.kTopFlywheelIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelMinOutputKey,
        RobotConfig.ShooterConfig.kTopFlywheelMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kTopFlywheelMaxOutputKey,
        RobotConfig.ShooterConfig.kTopFlywheelMaxOutput);
  }
  public void putBottomFlywheelOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard & test mode booleans
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelPGainKey, RobotConfig.ShooterConfig.kBottomFlywheelP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelIGainKey, RobotConfig.ShooterConfig.kBottomFlywheelI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelDGainKey, RobotConfig.ShooterConfig.kBottomFlywheelD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelFFGainKey, RobotConfig.ShooterConfig.kBottomFlywheelFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelIZoneKey, RobotConfig.ShooterConfig.kBottomFlywheelIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelMinOutputKey,
        RobotConfig.ShooterConfig.kBottomFlywheelMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kBottomFlywheelMaxOutputKey,
        RobotConfig.ShooterConfig.kBottomFlywheelMaxOutput);
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
      double pTopFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelPGainKey, 0);
      double iTopFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelIGainKey, 0);
      double dTopFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelDGainKey, 0);
      double izTopFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelIZoneKey, 0);
      double ffTopFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelFFGainKey, 0);
      double maxTopFlywheel =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelMaxOutputKey, 0);
      double minTopFlywheel =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kTopFlywheelMinOutputKey, 0);

          // read PID coefficients from SmartDashboard
      double pBottomFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelPGainKey, 0);
      double iBottomFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelIGainKey, 0);
      double dBottomFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelDGainKey, 0);
      double izBottomFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelIZoneKey, 0);
      double ffBottomFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelFFGainKey, 0);
      double maxBottomFlywheel =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelMaxOutputKey, 0);
      double minBottomFlywheel =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kBottomFlywheelMinOutputKey, 0);

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
      if (m_bottomFlywheelPidController.getP() != pBottomFlywheel) {
        m_bottomFlywheelPidController.setP(pBottomFlywheel);
      }
      if (m_bottomFlywheelPidController.getI() != iFBottomflywheel) {
        m_bottomFlywheelPidController.setI(iBottomFlywheel);
      }
      if (m_bottomFlywheelPidController.getD() != dBottomFlywheel) {
        m_bottomFlywheelPidController.setD(dBottomFlywheel);
      }
      if (m_bottomFlywheelPidController.getFF() != ffBottomFlywheel) {
        m_bottomFlywheelPidController.setFF(ffBottomFlywheel);
      }
      if (m_bottomFlywheelPidController.getIZone() != izBottomFlywheel) {
        m_bottomFlywheelPidController.setIZone(izBottomFlywheel);
      }
      if (m_bottomFlywheelPidController.getOutputMax() != maxBottomFlywheel
          || m_bottomFlywheelPidController.getOutputMin() != minBottomFlywheel) {
        m_bottomFlywheelPidController.setOutputRange(minBottomFlywheel, maxBottomFlywheel);
      }
    }
  }

  // sets the target angle the shooter should be at
  // should include motor, encoder, and pid controller for the angle motors
  public void setAngle(double targetAngleDegrees) {

  }

  // runs the rollers
  // should include the roller motors
  public void startFeedNote() {

  }

  // stops the rollers
  // should include the roller motors
  public void stopFeedNote() {

  }

  // runs the flywheel at a speed in rotations per minute
  // should include motor, encoder, and pid controller for the flywheel motors
  public void runFlywheel(double targetRPM) {

  }

  // extends shield
  // should include shield motor, possibly pid
  public void extendShield() {

  }

  // retracts shield
  // should include shield motor, possibly pid
  public void retractShield() {

  }
  /*
  public boolean hasNote() {
    return Intake.isIndexed();
  }
  */
}
