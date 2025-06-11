package frc.robot;

import java.lang.System.Logger.Level;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {
    /**
     * The LoggingConstants class holds the logging configuration for the robot.
     * 
     * <p>currentLogLevel: The current logging level for the robot's logging system.
     * The log level determines the severity of messages that will be logged.
     * Common log levels include DEBUG, INFO, WARN, and ERROR, with DEBUG being the most
     * verbose and ERROR being the least.
     */
    public static final class LoggingConstants{
        public static final Level CURRENT_LOG_LEVEL = Level.INFO;
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = .0952492;
        // gear ratio is inversed (1/gear ratio) so we can multiply instead of dividing
        // the Rot to meter funtion
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        // 2048 is pulses per rotation of the motor
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

        // CTRE mesures their velcity in 100ms, so we multiply it by 10 to get 1s
        public static final double kDriveEncoderRPMS2MeterPerSec = kDriveEncoderRot2Meter * 10;
        public static final double kTurningEncoderRPMS2RadPerSec = kTurningEncoderRot2Rad * 10;
        // use guess and check to find. when the module is overshooting this needs to be
        // fine tuned
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),//this is right dont mess it up
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 3;

        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 6;

        // Positive should be counter clockwise
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false; 
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;

        // Positive should be clockwise
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // To find set the motors forward record the value (don't inverse the value)
        // 180 degrees means the wheels are running backwards. Preferably set to 0
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = 0; 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = 0;

        // Max physical speed of our motors. Required for motor speed caculations
        // To find set the modules to 100% and see what speed cap out at
        public static final double kPhysicalMaxSpeedPercent = 1;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 2 * Math.PI;

    }

    public static final class TargetPosConstants {
        // Motion constants for target position drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedPercent / 1;//was 4 (changed 2/20/25);
        public static final double kMaxAngularSpeed = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 16;

        public static final double kForwardMaxAcceleration = .03;
        public static final double kBackwardMaxAcceleration = .075;
        public static final double kMaxAngularAcceleration = Math.PI / 3;
        public static final double kBackwardAngularAcceleration = -Math.PI * 9;

        public static final double kMinAngluarSpeedRadians = Math.PI / 16;
        public static final double kMinSpeedMetersPerSec = .2;

        public static final double kPDriveController = .1;
        public static final double kPAngleController = .05;
        public static final double kAcceptableDistanceError = 0.01;// subject to change was 0.04
        public static final double kAcceptableAngleError = 3;
    }

    public static final class OperatorConstants {
        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kXboxControllerPort = 2;

        // left joystick
        public static final int kLeftDriverYAxis = 1;
        public static final int kLeftDriverXAxis = 0;

        // right joystick
        public static final int kRightDriverRotAxis = 0;

        // area were joysticks will not activate
        public static final double kXboxControllerDeadband = 0.1;
        public static final double kFlightControllerDeadband = 0.05;
    }

    public static final class SensorConstants {
        public static final int kPigeonID = 0;

        public static final double sizeOfFieldMetersX = 17.548225;
        public static final double sizeOfFieldMetersY = 8.0518;
    }
    public final class VisionConstants {
        public static String kFrontLimelightName = null;//"limelight-side";
        public static String kSideLimelightName = "limelight-obj";
        public static double kMaxAllowedTagDistance = 5.0;
    }
}
