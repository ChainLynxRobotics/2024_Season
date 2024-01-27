package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeConfig;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_rollerFollowerMotor; // left motor (Rollers)
  private final CANSparkMax m_rollerLeaderMotor; // right motor (Rollers)

  private final SparkPIDController m_rollerPidController; // Roller velocity PID controllers
  private final RelativeEncoder m_rollerEncoder; // Roller Relative Encoder

  private final DigitalInput m_intakeSensor; // Line break note sensor

  private boolean m_testModeCheck1; // Booleans for if test mode is enabled
  private boolean m_testModeCheck2;

  // Constructs intake and initializes motor, PID, encoder objects, sensor
  public Intake() {
    m_rollerFollowerMotor = new CANSparkMax(IntakeConfig.kFollowerMotorID, MotorType.kBrushless);
    m_rollerLeaderMotor = new CANSparkMax(IntakeConfig.kLeaderMotorID, MotorType.kBrushless);
    m_rollerFollowerMotor.follow(this.m_rollerLeaderMotor);

    m_rollerPidController = m_rollerLeaderMotor.getPIDController();
    m_rollerEncoder = m_rollerLeaderMotor.getEncoder();
    zeroEncoders();

    m_intakeSensor = new DigitalInput(RobotConstants.IntakeConfig.kLineBreakSensor);

    // set Roller PID coefficients
    m_rollerPidController.setP(RobotConfig.IntakeConfig.kRollerP);
    m_rollerPidController.setI(RobotConfig.IntakeConfig.kRollerI);
    m_rollerPidController.setD(RobotConfig.IntakeConfig.kRollerD);
    m_rollerPidController.setFF(RobotConfig.IntakeConfig.kRollerFF);
    m_rollerPidController.setIZone(RobotConfig.IntakeConfig.kRollerIZone);
    m_rollerPidController.setOutputRange(
        RobotConfig.IntakeConfig.kRollerMinOutput, RobotConfig.IntakeConfig.kRollerMaxOutput);

    // display Roller PID coefficients on SmartDashboard & test mode booleans
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerPGainKey, RobotConfig.IntakeConfig.kRollerP);
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerIGainKey, RobotConfig.IntakeConfig.kRollerI);
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerDGainKey, RobotConfig.IntakeConfig.kRollerD);
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerFFGainKey, RobotConfig.IntakeConfig.kRollerFF);
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerIZoneKey, RobotConfig.IntakeConfig.kRollerIZone);
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerMinOutputKey, RobotConfig.IntakeConfig.kRollerMinOutput);
    SmartDashboard.putNumber(
        RobotConfig.IntakeConfig.kRollerMaxOutputKey, RobotConfig.IntakeConfig.kRollerMaxOutput);
    SmartDashboard.putBoolean(RobotConfig.IntakeConfig.kTestCheck1Key, m_testModeCheck1);
    SmartDashboard.putBoolean(RobotConfig.IntakeConfig.kTestCheck2Key, m_testModeCheck2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    boolean testCheck1 = SmartDashboard.getBoolean(RobotConfig.IntakeConfig.kTestCheck1Key, false);
    boolean testCheck2 = SmartDashboard.getBoolean(RobotConfig.IntakeConfig.kTestCheck2Key, false);

    // Checks if test mode is enabled
    if (m_testModeCheck1 != testCheck1) {
      m_testModeCheck1 = testCheck1;
    }
    if (m_testModeCheck2 != testCheck2) {
      m_testModeCheck2 = testCheck2;
    }

    if (m_testModeCheck1 && m_testModeCheck2) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerPGainKey, 0);
      double i = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerIGainKey, 0);
      double d = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerDGainKey, 0);
      double iz = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerIZoneKey, 0);
      double ff = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerFFGainKey, 0);
      double max = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerMaxOutputKey, 0);
      double min = SmartDashboard.getNumber(RobotConfig.IntakeConfig.kRollerMinOutputKey, 0);

      // checks PID values against Smartdash board
      if (m_rollerPidController.getP() != p) {
        m_rollerPidController.setP(p);
      }
      if (m_rollerPidController.getI() != i) {
        m_rollerPidController.setI(i);
      }
      if (m_rollerPidController.getD() != d) {
        m_rollerPidController.setD(d);
      }
      if (m_rollerPidController.getFF() != ff) {
        m_rollerPidController.setFF(ff);
      }
      if (m_rollerPidController.getIZone() != iz) {
        m_rollerPidController.setIZone(iz);
      }
      if (m_rollerPidController.getOutputMax() != max
          || m_rollerPidController.getOutputMin() != min) {
        m_rollerPidController.setOutputRange(min, max);
      }
    }
  }

  /**
   * Sets the target rpm
   *
   * @param targetRPM Target rpm
   */
  public void setTargetRPM(double targetRPM) {
    m_rollerPidController.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Runs the intake
   *
   * @param motorOutput Motor speed from -1.0 to 1.0 as a percentage
   */
  public void run(double motorOutput) {
    m_rollerLeaderMotor.set(motorOutput);
  }

  /**
   * Runs the intake at a default speed The motor can be reversed by inputing a true as its
   * parameter
   *
   * @param isReversed The input to reverse the motor speed
   */
  public void run(boolean isReversed) {
    double runSpeed =
        isReversed
            ? -RobotConfig.IntakeConfig.kDefaultSpeed
            : RobotConfig.IntakeConfig.kDefaultSpeed;
    run(runSpeed);
  }

  // Checks if note is indexed
  public boolean isIndexed() {
    return m_intakeSensor.get();
  }

  // Stops the motor
  public void stop() {
    m_rollerLeaderMotor.stopMotor();
  }

  // Zeros the encoder(s)
  public void zeroEncoders() {
    m_rollerEncoder.setPosition(0);
  }
}
