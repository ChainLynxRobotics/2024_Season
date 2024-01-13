package frc.robot.subsystems.drive;


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

import com.ctre.phoenix6.hardware.Pigeon2;


public class Drivetrain extends SubsystemBase {
<<<<<<< HEAD
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {}
=======

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft;
  private final MAXSwerveModule m_frontRight;
  private final MAXSwerveModule m_rearLeft;
  private final MAXSwerveModule m_rearRight;

  public Pigeon2 m_gyro;

  private final PowerDistribution powerDistribution;

  private double prevAngle;
  private double rightAngGoal;
  private double turnDir;

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation;
  private double currentTranslationDir;
  private double currentTranslationMag;

  private double headingOffset;

  private SlewRateLimiter magLimiter;
  private SlewRateLimiter rotLimiter;

  double prevTime;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  public Drivetrain() {
    m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

    m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

    m_rearLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);

    m_rearRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

    m_gyro = new Pigeon2(DriveConstants.kGyroId);

    powerDistribution = new PowerDistribution();

    magLimiter = new SlewRateLimiter(OIConstants.kMagnitudeSlewRate);
    rotLimiter = new SlewRateLimiter(OIConstants.kRotationalSlewRate);

    prevTime = WPIUtilJNI.now() * 1e-6;

    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
      }
    );

    powerDistribution.clearStickyFaults();
    SmartDashboard.putNumber("driveVelocity", 0);
  }
>>>>>>> 4cb4d8d (implemented drivetrain subsystem and basic drive command functionality from last year)

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
      }
    );

    double ang = m_gyro.getRotation2d().getDegrees();
    SmartDashboard.putNumber("delta heading", ang - prevAngle);

    prevAngle = ang;

    SmartDashboard.putNumber("heading", ang - headingOffset);

    SmartDashboard.putNumber("right stick angle", rightAngGoal);
    SmartDashboard.putNumber("turn direction", turnDir);

  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(
          Rotation2d.fromDegrees(-m_gyro.getAngle()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition(),
          },
          pose
      );
  }

  public void drive(
      double xSpeed,
      double ySpeed,
      double xRot,
      double yRot,
      boolean altDrive,
      boolean centerGyro
  ) {
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
      //convert to degrees
      rightAngGoal = Math.atan2(xRot, yRot) * 180 / Math.PI;
      if (xRot != 0 || yRot != 0) {
          //convert to degrees
          double stickAng = Math.atan2(xRot, yRot) * 180 / Math.PI;
          //gets the difference in angle, then uses mod to make sure its from -180 to 180
          rot =
              Math.tanh(
                  ((m_gyro.getAngle() + stickAng + 180) % 360 - 180) /
                  DriveConfig.altTurnSmoothing
              ) *
              DriveConfig.kMaxAngularSpeed;
      }
      turnDir = rot;
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
        double inputTranslationMag = Math.sqrt(
            Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)
        );

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (currentTranslationMag != 0.0) {
            directionSlewRate =
                Math.abs(
                    OIConstants.kDirectionSlewRate / currentTranslationMag
                );
        } else {
            directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
        }

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTime;
        double angleDif = SwerveUtils.AngleDifference(
            inputTranslationDir,
            currentTranslationDir
        );
        if (angleDif < 0.45 * Math.PI) {
            currentTranslationDir =
                SwerveUtils.StepTowardsCircular(
                    currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime
                );
            currentTranslationMag =
                magLimiter.calculate(inputTranslationMag);
        } else if (angleDif > 0.85 * Math.PI) {
            if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                // keep currentTranslationDir unchanged
                currentTranslationMag = magLimiter.calculate(0.0);
            } else {
                currentTranslationDir =
                    SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                currentTranslationMag =
                    magLimiter.calculate(inputTranslationMag);
            }
        } else {
            currentTranslationDir =
                SwerveUtils.StepTowardsCircular(
                    currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime
                );
            currentTranslationMag = magLimiter.calculate(0.0);
        }
        prevTime = currentTime;

        xSpeedCommanded =
            currentTranslationMag * Math.cos(currentTranslationDir);
        ySpeedCommanded =
            currentTranslationMag * Math.sin(currentTranslationDir);
        currentRotation = rotLimiter.calculate(rot);
    } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        currentRotation = rot;
    }
    
    // Adjust input based on max speed
    double xSpeedDelivered =
        xSpeedCommanded * DriveConfig.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered =
        ySpeedCommanded * DriveConfig.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConfig.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered,
            ySpeedDelivered,
            rotDelivered,
            Rotation2d.fromDegrees(-m_gyro.getAngle())
        )
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConfig.kMaxSpeedMetersPerSecond
    );
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    headingOffset = m_gyro.getAngle();
    m_gyro.reset();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
