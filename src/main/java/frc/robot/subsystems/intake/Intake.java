package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_angleMotor; // intake angle motor

  private final CANSparkMax m_rollerFollowerMotor; // left motor
  private final CANSparkMax m_rollerLeaderMotor; // right motor

  // Constructs intake, and initializes motor objects
  public Intake() {
    this.m_angleMotor = new CANSparkMax(IntakeConstants.kAngleMotorID, MotorType.kBrushless);
    this.m_rollerFollowerMotor =
        new CANSparkMax(IntakeConstants.kFollowerMotorID, MotorType.kBrushless);
    this.m_rollerLeaderMotor =
        new CANSparkMax(IntakeConstants.kLeaderMotorID, MotorType.kBrushless);
    this.m_rollerFollowerMotor.follow(this.m_rollerLeaderMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
