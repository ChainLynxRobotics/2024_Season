package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;

// IDs and stuff
public final class RobotConstants {
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class DriveConstants {
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kDriveDeadband = 0.06;

    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kFrontLeftTurningCanId = 5;

    public static final int kFrontRightDrivingCanId = 8;
    public static final int kFrontRightTurningCanId = 7;

    public static final int kRearLeftDrivingCanId = 4;
    public static final int kRearLeftTurningCanId = 3;

    public static final int kRearRightDrivingCanId = 2;
    public static final int kRearRightTurningCanId = 1;

    public static final int kGyroId = 15;

    // Chassis configuration
    public static final Measure<Distance> kTrackWidth = Units.Inches.of(26.5);

    // Distance between centers of right and left wheels on robot
    public static final Measure<Distance> kWheelBase = Units.Inches.of(26.5);

    public static final Measure<Distance> kWheelBaseRadius = edu.wpi.first.units.Units.Inches.of(31.75/2);

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase.in(Units.Meters) / 2, kTrackWidth.in(Units.Meters) / 2),
            new Translation2d(kWheelBase.in(Units.Meters) / 2, -kTrackWidth.in(Units.Meters) / 2),
            new Translation2d(-kWheelBase.in(Units.Meters) / 2, kTrackWidth.in(Units.Meters) / 2),
            new Translation2d(-kWheelBase.in(Units.Meters) / 2, -kTrackWidth.in(Units.Meters) / 2));

    public static final class OIConstants {

      public static final int kDriverControllerPort = 0;
      public static final double kDriveDeadband = 0.06;
      public static final double kMagnitudeDeadband = 0.06;
      public static final double kDirectionSlewRate = 10; // radians per second
      public static final double kMagnitudeSlewRate = 90; // percent per second (1 = 100%)
      public static final double kRotationalSlewRate = 90; // percent per second (1 = 100%)
    }

    // update these constants when we actually test this on the robot
    public static final class SwerveModuleConstants {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
      // This changes the drive speed of the module (a pinion gear with more teeth will result in a
      // robot that drives faster).
      public static final int kDrivingMotorPinionTeeth = 14;

      public static final Measure<Velocity<Distance>> kMaxModuleSpeed = Units.MetersPerSecond.of(1);

      // Invert the turning encoder, since the output shaft rotates in the opposite direction of
      // the steering motor in the MAXSwerve Module.
      public static final boolean kTurningEncoderInverted = true;

      // Calculations required for driving motor conversion factors and feed forward
      public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
      // bevel pinion
      public static final double kDrivingMotorReduction =
          (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps =
          (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

      public static final double kDrivingEncoderPositionFactor =
          (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
      public static final double kDrivingEncoderVelocityFactor =
          ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

      public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double kTurningEncoderVelocityFactor =
          (2 * Math.PI) / 60.0; // radians per second

      public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
      public static final double kTurningEncoderPositionPIDMaxInput =
          kTurningEncoderPositionFactor; // radians

      public static final double kDrivingP = 0.04;
      public static final double kDrivingI = 0;
      public static final double kDrivingD = 0;
      public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;

      public static final double kTurningP = 1;
      public static final double kTurningI = 0;
      public static final double kTurningD = 0;
      public static final double kTurningFF = 0;
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;

      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

      public static final int kDrivingMotorCurrentLimit = 50; // amps
      public static final int kTurningMotorCurrentLimit = 20; // amps
    }
  }

  public static final class IntakeConstants {

    // TODO: update placeholder digital input ID
    public static final int kLineBreakSensor = -1;

    // TODO: update placeholder roller Motor ID
    public static final int kMotorID = -1;
  }
}
