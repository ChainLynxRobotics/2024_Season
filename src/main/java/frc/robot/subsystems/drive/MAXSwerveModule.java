// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.RobotConstants.DriveConstants.SwerveModuleConstants;

public class MAXSwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this.m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    this.m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    this.m_drivingSparkMax.restoreFactoryDefaults();
    this.m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    this.m_drivingEncoder = this.m_drivingSparkMax.getEncoder();
    this.m_turningEncoder = this.m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    this.m_drivingPIDController = this.m_drivingSparkMax.getPIDController();
    this.m_turningPIDController = this.m_turningSparkMax.getPIDController();
    this.m_drivingPIDController.setFeedbackDevice(this.m_drivingEncoder);
    this.m_turningPIDController.setFeedbackDevice(this.m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    this.m_drivingEncoder.setPositionConversionFactor(
        SwerveModuleConstants.kDrivingEncoderPositionFactor);
        this.m_drivingEncoder.setVelocityConversionFactor(
        SwerveModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    this.m_turningEncoder.setPositionConversionFactor(
        SwerveModuleConstants.kTurningEncoderPositionFactor);
    this.m_turningEncoder.setVelocityConversionFactor(
        SwerveModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    this.m_turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    this.m_turningPIDController.setPositionPIDWrappingEnabled(true);
    this.m_turningPIDController.setPositionPIDWrappingMinInput(
        SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
    this.m_turningPIDController.setPositionPIDWrappingMaxInput(
        SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    this.m_drivingPIDController.setP(SwerveModuleConstants.kDrivingP);
    this.m_drivingPIDController.setI(SwerveModuleConstants.kDrivingI);
    this.m_drivingPIDController.setD(SwerveModuleConstants.kDrivingD);
    this.m_drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF);
    this.m_drivingPIDController.setOutputRange(
        SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    this.m_turningPIDController.setP(SwerveModuleConstants.kTurningP);
    this.m_turningPIDController.setI(SwerveModuleConstants.kTurningI);
    this.m_turningPIDController.setD(SwerveModuleConstants.kTurningD);
    this.m_turningPIDController.setFF(SwerveModuleConstants.kTurningFF);
    this.m_turningPIDController.setOutputRange(
        SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput);

    this.m_drivingSparkMax.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
    this.m_turningSparkMax.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
    this.m_drivingSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
    this.m_turningSparkMax.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    this.m_drivingSparkMax.burnFlash();
    this.m_turningSparkMax.burnFlash();

    this.m_chassisAngularOffset = chassisAngularOffset;
    this.m_desiredState.angle = new Rotation2d(this.m_turningEncoder.getPosition());
    this.m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
      this.m_drivingEncoder.getVelocity(),
        new Rotation2d(this.m_turningEncoder.getPosition() - this.m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      this.m_drivingEncoder.getPosition(),
        new Rotation2d(this.m_turningEncoder.getPosition() - this.m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(this.m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(this.m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    this.m_drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    this.m_turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        this.m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.m_drivingEncoder.setPosition(0);
  }
}
