package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final int kPigeonID = 0;

    // Inversions
    public static final boolean kInvertGyro = false;
    public static final boolean kInvertDrive = false;
    public static final boolean kInvertSteer = false;
    public static final boolean kInvertCancoder = false;
    // Physical properties
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kTrackLength = Units.inchesToMeters(18.5);
    public static final double kRobotWidth = Units.inchesToMeters(25 + 3.25*2);
    
    public static final double kMaxLinearSpeed = Units.feetToMeters(15.5);
    public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter*Math.PI;
    public static final double kDriveGearRatio = 6.12; // 6.12:1 L3 ratio
    public static final double kSteerGearRatio = 12.8; // 12.8:1
    public static final double kDriveDistPerPulse = kWheelCircumference / 1024 / kDriveGearRatio;
    public static final double kSteerRadPerPulse = 2 * Math.PI / 1024;

    public enum Module {
        FL( // Front left
            1,
            0, 0, 1,
            1, 2, 3, 96.855,
            kTrackLength/2, kTrackWidth/2
        ),
        FR( // Front Right
            2,
            2, 4, 5,
            3, 6, 7, -118.565,
            kTrackLength/2, -kTrackWidth/2
        ),
        BL( // Back Left
            3,
            4, 8, 9,
            5, 10, 11, -122.344,
            -kTrackLength/2, kTrackWidth/2
        ),
        BR( // Back Right
            4,
            6, 12, 13,
            7, 14, 15, 175.078,
            -kTrackLength/2, -kTrackWidth/2
        );

        public final int moduleNum;
        public final int driveMotorID;
        public final int driveEncoderA;
        public final int driveEncoderB;
        public final int steerMotorID;
        public final int steerEncoderA;
        public final int steerEncoderB;
        public final double angleOffset;
        public final Translation2d centerOffset;
        private Module(
            int moduleNum,
            int driveMotorID, int driveEncoderA, int driveEncoderB,
            int steerMotorID, int steerEncoderA, int steerEncoderB, double angleOffset,
            double xOffset, double yOffset
            ){
            this.moduleNum = moduleNum;
            this.driveMotorID = driveMotorID;
            this.driveEncoderA = driveEncoderA;
            this.driveEncoderB = driveEncoderB;
            this.steerMotorID = steerMotorID;
            this.steerEncoderA = steerEncoderA;
            this.steerEncoderB = steerEncoderB;
            this.angleOffset = angleOffset;
            centerOffset = new Translation2d(xOffset, yOffset);
        }
    }

    // Current limits
    public static final int kDriveContinuousCurrentLimit = 40;
    public static final int kDrivePeakCurrentLimit = 65;
    public static final double kDrivePeakCurrentDuration = 0.1;
    public static final int kSteerContinuousCurrentLimit = 25;
    public static final int kSteerPeakCurrentLimit = 40;
    public static final double kSteerPeakCurrentDuration = 0.1;
    // Voltage compensation
    public static final double kVoltageSaturation = 12;
    public static final int kVoltageMeasurementSamples = 32;

    // Feedforward
    // Linear drive feed forward
    public static final SimpleMotorFeedforward kDriveFF = new SimpleMotorFeedforward( // real
        0.2, // Voltage to break static friction
        2.25, // Volts per meter per second
        0.17 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward kSteerFF = new SimpleMotorFeedforward( // real
        0.5, // Voltage to break static friction
        0.23, // Volts per radian per second
        0.0056 // Volts per radian per second squared
    );

    // PID
    public static final double kDriveKP = 1;
    public static final double kDriveKI = 0;
    public static final double kDriveKD = 0;

    public static final double kSteerKP = 20;
    public static final double kSteerKI = 0;
    public static final double kSteerKD = 0.25;
    public static final double kSteerVelocity = Units.rotationsToRadians(7); // rotations per second
    public static final double kSteerAcceleration = Units.rotationsToRadians(40); // rotations per second squared
    public static final int kAllowableSteeringError = 80;

    public static final int kCANTimeout = 100;
}
