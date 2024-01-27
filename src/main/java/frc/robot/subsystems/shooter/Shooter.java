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

  private CANSparkMax m_flywheelMotor;

  private SparkPIDController m_anglePidController;
  private SparkPIDController m_flywheelPidController;

  private RelativeEncoder m_angleEncoder;
  private RelativeEncoder m_flywheelEncoder;

  private DigitalInput m_linebreakSensor;

  private boolean m_testModeCheck1; // Booleans for if test mode is enabled
  private boolean m_testModeCheck2;

  public Shooter() {
    m_rollerMotorLeft = new CANSparkMax(ShooterConstants.kRollerMotorLeftId, MotorType.kBrushless);
    m_rollerMotorRight =
        new CANSparkMax(ShooterConstants.kRollerMotorRightId, MotorType.kBrushless);

    m_flywheelMotor = new CANSparkMax(ShooterConstants.kFlywheelMotorId, MotorType.kBrushed);
    m_flywheelPidController = m_flywheelMotor.getPIDController();
    m_flywheelEncoder = m_flywheelMotor.getEncoder();

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

    // set Flywheel PID coefficients
    m_flywheelPidController.setP(RobotConfig.ShooterConfig.kFlywheelP);
    m_flywheelPidController.setI(RobotConfig.ShooterConfig.kFlywheelI);
    m_flywheelPidController.setD(RobotConfig.ShooterConfig.kFlywheelD);
    m_flywheelPidController.setFF(RobotConfig.ShooterConfig.kFlywheelFF);
    m_flywheelPidController.setIZone(RobotConfig.ShooterConfig.kFlywheelIZone);
    m_flywheelPidController.setOutputRange(
        RobotConfig.ShooterConfig.kFlywheelMinOutput, RobotConfig.ShooterConfig.kFlywheelMaxOutput);

    putAngleOnSmartDashboard();
    putFlywheelOnSmartDashboard();

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

  public void putFlywheelOnSmartDashboard() {
    // display Angle PID coefficients on SmartDashboard & test mode booleans
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelPGainKey, RobotConfig.ShooterConfig.kFlywheelP);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelIGainKey, RobotConfig.ShooterConfig.kFlywheelI);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelDGainKey, RobotConfig.ShooterConfig.kFlywheelD);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelFFGainKey, RobotConfig.ShooterConfig.kFlywheelFF);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelIZoneKey, RobotConfig.ShooterConfig.kFlywheelIZone);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelMinOutputKey,
        RobotConfig.ShooterConfig.kFlywheelMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.ShooterConfig.kFlywheelMaxOutputKey,
        RobotConfig.ShooterConfig.kFlywheelMaxOutput);
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
      double pFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelPGainKey, 0);
      double iFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelIGainKey, 0);
      double dFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelDGainKey, 0);
      double izFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelIZoneKey, 0);
      double ffFlywheel = SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelFFGainKey, 0);
      double maxFlywheel =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelMaxOutputKey, 0);
      double minFlywheel =
          SmartDashboard.getNumber(RobotConfig.ShooterConfig.kFlywheelMinOutputKey, 0);

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
      if (m_flywheelPidController.getP() != pFlywheel) {
        m_flywheelPidController.setP(pFlywheel);
      }
      if (m_flywheelPidController.getI() != iFlywheel) {
        m_flywheelPidController.setI(iFlywheel);
      }
      if (m_flywheelPidController.getD() != dFlywheel) {
        m_flywheelPidController.setD(dFlywheel);
      }
      if (m_flywheelPidController.getFF() != ffFlywheel) {
        m_flywheelPidController.setFF(ffFlywheel);
      }
      if (m_flywheelPidController.getIZone() != izFlywheel) {
        m_flywheelPidController.setIZone(izFlywheel);
      }
      if (m_flywheelPidController.getOutputMax() != maxFlywheel
          || m_flywheelPidController.getOutputMin() != minFlywheel) {
        m_flywheelPidController.setOutputRange(minFlywheel, maxFlywheel);
      }
    }
  }

  /**
   * runs the feed motors to feed the note to the shoooter flywheel and shoots the note
   *
   * @param speed the speed to run the feed motors from -1 to 1 where -1 is full reverse and 1 is
   *     full forward
   */
  public void run(double speed, CANSparkMax motor) {
    motor.set(speed);
  }

  /** runs the feed */
  public void runAmp() {
    run(1, m_flywheelMotor);
  }

  public boolean hasNote() {
    return m_linebreakSensor.get();
  }
}
