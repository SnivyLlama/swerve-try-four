package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

// basically all the "problems" lie in this file
public final class Constants {
    // Joystick axes
    public static final int kAxisX = 0;
    public static final int kAxisY = 1;
    public static final int kAxisZ = 4;
    // TODO: double check values
    public static final double kRobotWidth = Units.inchesToMeters(25);
    public static final double kRobotDepth = Units.inchesToMeters(25);

    // TODO: use sysid for these
    public static final double kVSteer = 2.0;
    public static final double kASteer = 0.1;
    public static final double kVDrive = 2.0;
    public static final double kADrive = 0.1;

    // note: this doesn't mean anything right now except for joystick inputs
    public static final double kMaxVelocity = 2.5;

    // TODO: find gearings and radius
    public static final double kSteerGearing = 21.5;
    public static final double kDriveGearing = 1;
    public static final double kWheelRadius = Units.inchesToMeters(2.0);
    public static final double kWheelCircum = 2 * Math.PI * kWheelRadius;

    // CAN IDs
    public static final int kFRDriveId = 2;
    public static final int kFLDriveId = 7;
    public static final int kBRDriveId = 4;
    public static final int kBLDriveId = 5;

    public static final int kFRSteerId = 1;
    public static final int kFLSteerId = 8;
    public static final int kBRSteerId = 3;
    public static final int kBLSteerId = 6;

    public static final boolean kCosineScale = true;

    // Gyroscope axes
    public static final IMUAxis kYawAxis = IMUAxis.kZ;
    // the other two dont matter as much
    public static final IMUAxis kPitchAxis = IMUAxis.kX;
    public static final IMUAxis kRollAxis = IMUAxis.kY;

    // Starting poses
    public static final Pose2d kZero = Pose2d.kZero;
    public static final Pose2d kRedStart = new Pose2d(
        Units.inchesToMeters(297.5), Units.inchesToMeters(158.5/2), Rotation2d.k180deg);
    public static final Pose2d kBlueStart = new Pose2d(
        Units.inchesToMeters(297.5+96.0), Units.inchesToMeters(158.5+146.5/2), Rotation2d.kZero);
    
    // PID constants
    public static final double kPSteer = 1.0;
    public static final double kISteer = 0.0;
    public static final double kDSteer = 0.1;
    public static final double kFFSteer = 0.0;

    public static final double kPDrive = 0.05;
    //public static final double kPDrive = 0.82;
    //public static final double kPDrive = 0.0;
    public static final double kIDrive = 0.0;
    public static final double kDDrive = 0.0;
    public static final double kFFDrive = 0.171;
    //public static final double kFFDrive = 2.0;
    //public static final double kFFDrive = 0.0;

    // The one solo simulation variable for now
    public static final double kSimNoise = 1.1e-5;
}
