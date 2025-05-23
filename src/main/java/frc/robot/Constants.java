package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

// basically all the "problems" lie in this file
public final class Constants {
    // Joystick axes
    public static final int kAxisX = 0;
    public static final int kAxisY = 1;
    public static final int kAxisZ = 4;
    public static final int kAxisA = 5; // for use with gyro
    // TODO: double check values
    public static final double kRobotTrackWidth = Units.inchesToMeters(25);
    public static final double kRobotTrackDepth = Units.inchesToMeters(25);
    // add on bumpers
    public static final double kRobotDepth = kRobotTrackDepth + Units.inchesToMeters(3);

    // TODO: use sysid for these
    public static final double kVSteer = 2.0;
    public static final double kASteer = 0.1;
    public static final double kVDrive = 2.0;
    public static final double kADrive = 0.1;

    // note: this doesn't mean anything right now except for joystick inputs
    public static final double kMaxVelocity = 4.0;
    public static final double kMaxAcceleration = 2.5;
    public static final double kMaxAngularVelocity = Units.degreesToRadians(180);
    public static final double kMaxAngularAcceleration = Units.degreesToRadians(120);

    // TODO: find gearings and radius
    public static final double kSteerGearing = 21.5;
    public static final double kDriveGearing = 1;
    public static final Distance kWheelRadius = Inches.of(2.0);
    public static final Distance kWheelCircum = kWheelRadius.times(2*Math.PI);

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

    // Absolute heading mode for joystick control
    public static final boolean kUseHeading = false;
    public static final Rotation2d kBlueOffset = Rotation2d.kZero;
    public static final Rotation2d kRedOffset = Rotation2d.k180deg;
    public static final Rotation2d kElasticOffset = Rotation2d.kCCW_Pi_2;

    // Gyroscope axes
    public static final IMUAxis kYawAxis = IMUAxis.kZ;
    // the other two dont matter as much
    public static final IMUAxis kPitchAxis = IMUAxis.kX;
    public static final IMUAxis kRollAxis = IMUAxis.kY;

    // Starting poses
    public static final Pose2d kZero = Pose2d.kZero;
    public static final Pose2d kRedStart = new Pose2d(
        Units.inchesToMeters(297.5+96.0), Units.inchesToMeters(158.5/2), Rotation2d.k180deg);
    public static final Pose2d kBlueStart = new Pose2d(
        Units.inchesToMeters(297.5), Units.inchesToMeters(158.5+146.5/2), Rotation2d.kZero);
    
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

    public static final double kPGyro = 3.2;
    public static final double kIGyro = 0.0;
    public static final double kDGyro = 0.0;

    // The one solo simulation variable for now
    public static final double kSimNoise = 1.1e-5;
}
