package frc.robot.constants;
//variable things like default speeds
public class RobotConfig {

    public static class DriveConfig {

        //4.45 m/s max speed
        public static final double kMaxSpeedBase = 4.8;
        public static final double kMaxSpeedScaleFactor = 0.9;
        public static final double kMaxSpeedMetersPerSecond =
            kMaxSpeedBase * kMaxSpeedScaleFactor;

        public static final double kMaxAngularSpeedBase = Math.PI;
        public static final double kMaxAngularSpeedScaleFactor = 0.7;
        public static final double kMaxAngularSpeed =
            kMaxAngularSpeedBase * kMaxAngularSpeedScaleFactor; // radians per second

        public static final double kFrontLeftChassisAngularOffset = 0.0;
        public static final double kFrontRightChassisAngularOffset = 0.0;
        public static final double kBackLeftChassisAngularOffset = 0.0;
        public static final double kBackRightChassisAngularOffset = 0.0;
        //scaling factor for the alternative turning mode
        public static final int altTurnSmoothing = 20;


    }

}
