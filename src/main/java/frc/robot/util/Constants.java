// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // Swerve modules
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14; // old value 1 / 5.8462
        public static final double kTurningMotorGearRatio = 1 / 12.8; // old value 1 / 18.0
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        // Used in working code currently
        public static final double kPTurning = 0.5;

        // These two used for simulation currently 
        public static final double kITurning = 0.0;
        public static final double kDTurning = 0.005;
        
    }

    // Swerve drive
    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(21); // 15.5

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(21.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(

        // Default value
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //fl
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //fr
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //bl
            new Translation2d(kWheelBase / 2, kTrackWidth / 2)); //br

        /* new Translation2d(kWheelBase / 2, kTrackWidth / 2), //fl
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //fr
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //bl
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //br
        */
                                                      // Driving Motor Ports
        public static final int kFrontLeftDriveMotorPort =  1;  // Front Left 
        public static final int kFrontRightDriveMotorPort = 3; // Front Right
        public static final int kBackRightDriveMotorPort =  5;  // Back Right
        public static final int kBackLeftDriveMotorPort =   7;   // Back Left

                                                                // Turning Motor Ports
        public static final int kFrontLeftTurningMotorPort = 2; // Front Left
        public static final int kFrontRightTurningMotorPort = 4;// Front Right
        public static final int kBackRightTurningMotorPort = 6; // Back Right
        public static final int kBackLeftTurningMotorPort = 8;  // Back Left

        // Encoder on NEO turning
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        // Encoder for NEO drive
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        // Absolute encoders reversed
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                                        // Need to update values for our specific magnet fields
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.98 * 2 * Math.PI; 
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = (.25+0.0918)  * 2 * Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = (0.0141+.25) * 2 * Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  0.2577  * 2 * Math.PI;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4 ;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    // Autonomous
    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 15;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    // Input and Output 
    public static final class IOConstants {

        public static final double kDeadband = 0.05;

        public static final int kLeftJoystick = 0;
        public static final int kRightJoystick = 1;
        public static final int kXboxController = 3;

        public static final int kFieldOrientedButton = 3;
        public static final int kZeroHeadingButton = 2;
        public static final int kRotatorButton = 3;
        public static final double kTransmitterOffset = 1.429;
           
    }

    // Vision
    public static final class VisionConstants{

        // The difference in height between the target's height and the height of the camera.
        public static final int deltaHeight = 0; 
        public static final int cameraAngle = 45;
    }
}
