package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig.DriveConfig;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.utils.SwerveUtils;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft;
  private final MAXSwerveModule m_frontRight;
  private final MAXSwerveModule m_rearLeft;
  private final MAXSwerveModule m_rearRight;

  private Pigeon2 m_gyro;

  private final PowerDistribution m_powerDistribution;

  private double m_prevAngle;
  private double m_rightAngGoal;
  private double m_turnDir;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation;
  private double m_currentTranslationDir;
  private double m_currentTranslationMag;

  private double m_headingOffset;

  private SlewRateLimiter m_magLimiter;
  private SlewRateLimiter m_rotLimiter;

  private double m_prevTime;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  public Drivetrain() {
    this.m_frontLeft =
        new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    this.m_frontRight =
        new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    this.m_rearLeft =
        new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    this.m_rearRight =
        new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    this.m_gyro = new Pigeon2(DriveConstants.kGyroId);

    this.m_powerDistribution = new PowerDistribution();

    this.m_magLimiter = new SlewRateLimiter(OIConstants.kMagnitudeSlewRate);
    this.m_rotLimiter = new SlewRateLimiter(OIConstants.kRotationalSlewRate);

    this.m_prevTime = WPIUtilJNI.now() * 1e-6;

    this.m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
              this.m_frontLeft.getPosition(),
              this.m_frontRight.getPosition(),
              this.m_rearLeft.getPosition(),
              this.m_rearRight.getPosition(),
            });

    this.m_powerDistribution.clearStickyFaults();
    SmartDashboard.putNumber("driveVelocity", 0);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          this.m_frontLeft.getPosition(),
          this.m_frontRight.getPosition(),
          this.m_rearLeft.getPosition(),
          this.m_rearRight.getPosition(),
        });

    double ang = this.m_gyro.getRotation2d().getDegrees();
    SmartDashboard.putNumber("delta heading", ang - this.m_prevAngle);

    this.m_prevAngle = ang;

    SmartDashboard.putNumber("heading", ang - this.m_headingOffset);

    SmartDashboard.putNumber("right stick angle", this.m_rightAngGoal);
    SmartDashboard.putNumber("turn direction", this.m_turnDir);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    this.m_odometry.resetPosition(
        Rotation2d.fromDegrees(-this.m_gyro.getAngle()),
        new SwerveModulePosition[] {
          this.m_frontLeft.getPosition(),
          this.m_frontRight.getPosition(),
          this.m_rearLeft.getPosition(),
          this.m_rearRight.getPosition(),
        },
        pose);
  }

  public void drive(
      double xSpeed,
      double ySpeed,
      double xRot,
      double yRot,
      boolean altDrive,
      boolean centerGyro) {
    if (centerGyro) zeroHeading();
    if (altDrive) {
      altDrive(xSpeed, ySpeed, xRot, yRot);
    } else {
      mainDrive(xSpeed, ySpeed, xRot);
    }
  }

  public void mainDrive(double xSpeed, double ySpeed, double xRot) {
    double rot = xRot * DriveConfig.kMaxAngularSpeed;
    move(xSpeed, ySpeed, rot);
  }

  public void altDrive(double xSpeed, double ySpeed, double xRot, double yRot) {
    double rot = 0;
    // convert to degrees
    this.m_rightAngGoal = Math.atan2(xRot, yRot) * 180 / Math.PI;
    if (xRot != 0 || yRot != 0) {
      // convert to degrees
      double stickAng = Math.atan2(xRot, yRot) * 180 / Math.PI;
      // gets the difference in angle, then uses mod to make sure its from -180 to 180
      rot =
          Math.tanh(
                  ((m_gyro.getAngle() + stickAng + 180) % 360 - 180) / DriveConfig.altTurnSmoothing)
              * DriveConfig.kMaxAngularSpeed;
    }
    this.m_turnDir = rot;
    move(xSpeed, ySpeed, rot);
  }

  private void move(double xSpeed, double ySpeed, double rot) {
    move(xSpeed, ySpeed, rot, true);
  }

  private void move(double xSpeed, double ySpeed, double rot, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (this.m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(OIConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif =
          SwerveUtils.AngleDifference(inputTranslationDir, this.m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        this.m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                this.m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        this.m_currentTranslationMag = this.m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (this.m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          this.m_currentTranslationMag = this.m_magLimiter.calculate(0.0);
        } else {
          this.m_currentTranslationDir =
              SwerveUtils.WrapAngle(this.m_currentTranslationDir + Math.PI);
          this.m_currentTranslationMag = this.m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        this.m_currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                this.m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        this.m_currentTranslationMag = this.m_magLimiter.calculate(0.0);
      }
      this.m_prevTime = currentTime;

      xSpeedCommanded = this.m_currentTranslationMag * Math.cos(this.m_currentTranslationDir);
      ySpeedCommanded = this.m_currentTranslationMag * Math.sin(this.m_currentTranslationDir);
      this.m_currentRotation = this.m_rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      this.m_currentRotation = rot;
    }

    // Adjust input based on max speed
    double xSpeedDelivered = xSpeedCommanded * DriveConfig.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConfig.kMaxSpeedMetersPerSecond;
    double rotDelivered = this.m_currentRotation * DriveConfig.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-this.m_gyro.getAngle())));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConfig.kMaxSpeedMetersPerSecond);
    this.m_frontLeft.setDesiredState(swerveModuleStates[0]);
    this.m_frontRight.setDesiredState(swerveModuleStates[1]);
    this.m_rearLeft.setDesiredState(swerveModuleStates[2]);
    this.m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.m_headingOffset = this.m_gyro.getAngle();
    this.m_gyro.reset();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
