package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConfig;
import frc.robot.constants.RobotConfig.DriveConfig;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotConstants.DriveConstants.OIConstants;
import frc.utils.SwerveUtils;
import frc.utils.Vector;

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
  private double m_prevSlewRateTime;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;
  private Pose2d m_pose;
  private ChassisSpeeds m_relativeSpeeds;

  private SwerveModulePosition[] m_swerveModulePositions;

  /** constructs a new Drivetrain object */
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

    // TODO: initialize this to where we place the robot on the field
    m_pose = new Pose2d();

    m_swerveModulePositions =
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        };

    m_gyro = new Pigeon2(DriveConstants.kGyroId);
    m_gyro.reset();

    m_timer = new Timer();

    m_powerDistribution = new PowerDistribution();

    m_magLimiter = new SlewRateLimiter(OIConstants.kMagnitudeSlewRate);
    m_rotLimiter = new SlewRateLimiter(OIConstants.kRotationalSlewRate);

    m_timer.start();
    m_prevSlewRateTime = m_timer.get();

    m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromRadians(-getGyroAngle().in(Units.Radians)),
            m_swerveModulePositions,
            m_pose);

    configureAutoBuilder();

    m_powerDistribution.clearStickyFaults();
    SmartDashboard.putNumber("driveVelocity", 0);
  }

  /** configures the pathplanner AutoBuilder */
  private void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getSpeeds,
        this::driveChassisSpeeds,
        RobotConfig.DriveConfig.kPathFollowerConfig,
        this::allianceCheck,
        this);
  }

  /**
   * returns the current speed of the drivetrain
   *
   * @return the current speed of the drivetrain
   */
  public ChassisSpeeds getSpeeds() {
    return m_relativeSpeeds;
  }

  /** stops the drivetrain's movement */
  public void stop() {
    move(new Vector(0, 0), 0);
  }

  /** runs the periodic functionality of the drivetrain */
  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), m_swerveModulePositions);
    double ang = getGyroAngle().in(Units.Radians);
    SmartDashboard.putNumber("delta heading", ang - m_prevAngleRadians);

    m_prevAngleRadians = ang;
    m_relativeSpeeds = getRobotRelativeSpeeds();
    m_pose = m_odometry.getPoseMeters();

    SmartDashboard.putNumber("heading", ang - m_headingOffsetRadians);

    SmartDashboard.putNumber("right stick angle", m_rightAngGoalRadians);
    SmartDashboard.putNumber("turn direction", m_turnDirRadians);
  }

  /**
   * Resets the pose estimator to the specified pose.
   *
   * @param pose The pose to which to set the estimator.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromRadians(-getGyroAngle().in(Units.Radians)),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition(),
        },
        pose);
  }

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
  public void drive(Vector spdVec, Vector rotVec, boolean altDrive, boolean centerGyro) {
    if (centerGyro) zeroHeading();
    if (altDrive) {
      altDrive(spdVec, rotVec);
    } else {
      mainDrive(spdVec, rotVec.x());
    }
  }

  /**
   * moves the divetrain based on the given ChassisSpeeds
   *
   * @param spds the target speeds of the drivetrain chassis
   */
  public void driveChassisSpeeds(ChassisSpeeds spds) {
    Vector spd = new Vector(spds.vxMetersPerSecond, spds.vyMetersPerSecond);
    double angVel = spds.omegaRadiansPerSecond;
    move(spd, angVel);
  }

  /**
   * moves the drivetrain using the main turning mode
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param xRot the speed to rotate with (-1, 1)
   */
  public void mainDrive(Vector spdVec, double xRot) {
    double rot = xRot * DriveConfig.kMaxAngularSpeed;
    move(spdVec, rot);
  }

  /**
   * gets the value of the robot's gyro as a Measure<Angle>
   *
   * @see Measure
   * @return the angle of the robot gyro
   */
  public Measure<Angle> getGyroAngle() {
    return Units.Degrees.of(m_gyro.getAngle());
  }

  /**
   * moves the drivetrain using the alternative turning mode
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param xRot the x component of the direction vector to point towards
   * @param yRot the y component of the direction vector to point towards
   */
  public void altDrive(Vector spdVec, Vector rotVec) {
    double rot = 0;
    m_rightAngGoalRadians = rotVec.angle();
    if (rotVec.squaredMag() > 0) {
      double stickAng = m_rightAngGoalRadians;
      // gets the difference in angle, then uses mod to make sure its from -PI rad to PI rad
      rot = altTurnSmooth(stickAng);
    }
    m_turnDirRadians = rot;
    move(spdVec, rot);
  }

  /**
   * returns the current speed of the robot from it's reference frame
   *
   * @return the current speed of the robot from it's reference frame
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
        });
  }

  /**
   * applies smoothing to the turning input of altDrive
   *
   * @param stickAng the given angle of the driver turning stick
   * @return the commanded rotation based on the rotation input
   */
  private double altTurnSmooth(double stickAng) {
    return Math.tanh(
            ((getGyroAngle().in(Units.Radians) + stickAng + Math.PI) % (2 * Math.PI) - Math.PI)
                / DriveConfig.altTurnSmoothing)
        * DriveConfig.kMaxAngularSpeed;
  }

  /**
   * returns the current position of the robot on the field
   *
   * @return the current position of the robot on the field
   */
  private Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    return (pose);
  }

  /**
   * moves the drivetrain using the given values
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param rot the angular velocity to rotate the drivetrain in radians/s
   */
  public void move(Vector spdVec, double rot) {
    move(spdVec, rot, true);
  }

  /**
   * moves the drivetrain using the given values
   *
   * @param xSpeed the proportion of the robot's max velocity to move in the x direction
   * @param ySpeed the proportion of the robot's max velocity to move in the y direction
   * @param rot the angular velocity to rotate the drivetrain in radians/s
   * @param rateLimit whether or not to use slew rate limiting
   */
  private void move(Vector spdVec, double rot, boolean rateLimit) {
    spdVec = spdVec.copy();

    m_currentRotationRadians = rot;

    // TODO slew rate limiter disabled; must find fix (something to do with vector implementation)
    if (
    /*rateLimit*/ false) {
      spdVec = limitDirectionSlewRate(spdVec);
      m_currentRotationRadians = m_rotLimiter.calculate(rot);
    }

    // Adjust input based on max speed
    Vector spdDelivered = spdVec.copy().mult(DriveConfig.kMaxSpeedMetersPerSecond);
    double rotDelivered = m_currentRotationRadians * DriveConfig.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                spdDelivered.x(),
                spdDelivered.y(),
                rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle())));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConfig.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * applies slewrate limiting to the given control vector
   *
   * @param spdVec the vector which represents the commanded speed of the drivetrain
   * @return the slew rate limited Vector for controlling the drivetrain
   */
  private Vector limitDirectionSlewRate(Vector spdVec) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = spdVec.angle();
    double inputTranslationMag = spdVec.mag();

    // Calculate the direction slew rate based on an estimate of the lateral acceleration
    double directionSlewRate;
    if (!SwerveUtils.approxEqual(m_currentTranslationMag, 0)) {
      directionSlewRate = Math.abs(OIConstants.kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate =
          DriveConfig
              .HIGH_DIRECTION_SLEW_RATE; // some high number that means the slew rate is effectively
      // instantaneous
    }

    double currentTime = m_timer.get();
    double elapsedTime = currentTime - m_prevSlewRateTime;

    double angleDif =
        SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDirRadians);

    m_prevSlewRateTime = currentTime;

    if (angleDif < DriveConfig.MIN_ANGLE_SLEW_RATE) {
      m_currentTranslationDirRadians =
          SwerveUtils.StepTowardsCircular(
              m_currentTranslationDirRadians, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      return (new Vector(m_currentTranslationMag, 0)).rot(m_currentTranslationDirRadians);
    }

    if (angleDif > DriveConfig.MAX_ANGLE_SLEW_RATE) {
      if (SwerveUtils.approxEqual(
          m_currentTranslationMag,
          0)) { // some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
        return (new Vector(m_currentTranslationMag, 0)).rot(m_currentTranslationDirRadians);
      }

      m_currentTranslationDirRadians =
          SwerveUtils.WrapAngle(m_currentTranslationDirRadians + Math.PI);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      return (new Vector(m_currentTranslationMag, 0)).rot(m_currentTranslationDirRadians);
    }

    m_currentTranslationDirRadians =
        SwerveUtils.StepTowardsCircular(
            m_currentTranslationDirRadians, inputTranslationDir, directionSlewRate * elapsedTime);

    m_currentTranslationMag = m_magLimiter.calculate(0.0);

    return (new Vector(m_currentTranslationMag, 0)).rot(m_currentTranslationDirRadians);
  }

  /**
   * checks whether pathplanner paths should be flipped based on the current alliance
   *
   * @return whether pathplanner paths should be flipped
   */
  private boolean allianceCheck() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_headingOffsetRadians = getGyroAngle().in(Units.Radians);
    m_gyro.reset();
  }
}
