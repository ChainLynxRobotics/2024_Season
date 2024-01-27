package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig.DriveConfig;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.utils.SwerveUtils;

/** an object representing the Drivetrain of a swerve drive frc robot */
public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft;
  private final MAXSwerveModule m_frontRight;
  private final MAXSwerveModule m_rearLeft;
  private final MAXSwerveModule m_rearRight;

  private Pigeon2 m_gyro;

  private final PowerDistribution m_powerDistribution;

  private double m_prevAngleRadians;
  private double m_rightAngGoalRadians;
  private double m_turnDirRadians;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotationRadians;
  private double m_currentTranslationDirRadians;
  private double m_currentTranslationMag;

  private double m_headingOffsetRadians;

  private SlewRateLimiter m_magLimiter;
  private SlewRateLimiter m_rotLimiter;

  private Timer m_timer;
  private double m_prevTime;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  /** constructs a new Drivatrain object */
  public Drivetrain() {
    m_frontLeft =
        new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    m_frontRight =
        new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    m_rearLeft =
        new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    m_rearRight =
        new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    m_gyro = new Pigeon2(DriveConstants.kGyroId);

    m_timer = new Timer();

    m_powerDistribution = new PowerDistribution();

    m_magLimiter = new SlewRateLimiter(OIConstants.kMagnitudeSlewRate);
    m_rotLimiter = new SlewRateLimiter(OIConstants.kRotationalSlewRate);

    m_timer.start();
    m_prevTime = m_timer.get();

    m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromRadians(-getGyroRadians()),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition(),
            });

    m_powerDistribution.clearStickyFaults();
    SmartDashboard.putNumber("driveVelocity", 0);
  }

  /** runs the periodic functionality of the drivetrain */
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromRadians(-getGyroRadians()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
        });

    double ang = getGyroRadians();
    SmartDashboard.putNumber("delta heading", ang - m_prevAngleRadians);

    m_prevAngleRadians = ang;

    SmartDashboard.putNumber("heading", ang - m_headingOffsetRadians);

    SmartDashboard.putNumber("right stick angle", m_rightAngGoalRadians);
    SmartDashboard.putNumber("turn direction", m_turnDirRadians);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-getGyroRadians()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
        },
        pose);
  }

  // TODO: any x and y component values should use vectors instead for conciseness and readability

  /**
   * drives the drivatrain using the given inputs the magnitude of the joystick components shouldn't
   * be > 1 (x^2 + y^2 <= 1)
   *
   * @param xSpeed the x-pos of the left joystick (-1, 1)
   * @param ySpeed the y-pos of the left joystick (-1, 1)
   * @param xRot the x-pos of the right joystick (-1, 1)
   * @param yRot the x-pos of the right joystick (-1, 1)
   * @param altDrive whether or not to use the alternative turning mode
   * @param centerGyro whether or not to reset the gyro position to the current rotation
   */
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

  /**
   * moves the drivetrain using the main turning mode
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param xRot the speed to rotate with (-1, 1)
   */
  private void mainDrive(double xSpeed, double ySpeed, double xRot) {
    double rot = xRot * DriveConfig.kMaxAngularSpeed;
    move(xSpeed, ySpeed, rot);
  }

  /**
   * gets the value of the robot's gyro in radians
   *
   * @return the angle of the robot gyro in radians
   */
  private double getGyroRadians() {
    return Units.degreesToRadians(m_gyro.getAngle());
  }

  /**
   * moves the drivetrain using the alternative turning mode
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param xRot the x component of the direction vector to point towards
   * @param yRot the y component of the direction vector to point towards
   */
  private void altDrive(double xSpeed, double ySpeed, double xRot, double yRot) {
    double rot = 0;
    m_rightAngGoalRadians = Math.atan2(xRot, yRot);
    if (Double.compare(xRot, 0) != 0 || Double.compare(yRot, 0) != 0) {
      double stickAng = Math.atan2(xRot, yRot);
      // gets the difference in angle, then uses mod to make sure its from -PI rad to PI rad
      rot = altTurnSmooth(stickAng);
          
    }
    m_turnDirRadians = rot;
    move(xSpeed, ySpeed, rot);
  }

  private double altTurnSmooth(double stickAng){
    return Math.tanh(((getGyroRadians() + stickAng + Math.PI) % (2 * Math.PI) - Math.PI) / DriveConfig.altTurnSmoothing) * DriveConfig.kMaxAngularSpeed;
  }

  /**
   * moves the drivetrain using the given values
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param rot the angular velocity to rotate the drivetrain in radians/s
   */
  private void move(double xSpeed, double ySpeed, double rot) {
    move(xSpeed, ySpeed, rot, true);
  }

  /**
   * moves the drivetrain using the given values
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param rot the angular velocity to rotate the drivetrain in radians/s
   * @param rateLimit whether or not to use slew rate limiting
   */
  private void move(double xSpeed, double ySpeed, double rot, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(OIConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate =
            500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = m_timer.get();
      double elapsedTime = currentTime - m_prevTime;
      double angleDif =
          SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDirRadians);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDirRadians =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDirRadians,
                inputTranslationDir,
                directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDirRadians =
              SwerveUtils.WrapAngle(m_currentTranslationDirRadians + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDirRadians =
            SwerveUtils.StepTowardsCircular(
                m_currentTranslationDirRadians,
                inputTranslationDir,
                directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDirRadians);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDirRadians);
      m_currentRotationRadians = m_rotLimiter.calculate(rot);
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotationRadians = rot;
    }

    // Adjust input based on max speed
    double xSpeedDelivered = xSpeedCommanded * DriveConfig.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConfig.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotationRadians * DriveConfig.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle())));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConfig.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_headingOffsetRadians = getGyroRadians();
    m_gyro.reset();
  }

  /** run periodically when being simulated, required but not used in this implementation */
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
