package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {

    public static final double kRobotWidthMeters = 0;
    public static final double kTrackWidthXMeters = 0;
    public static final double kTrackWidthYMeters = 0;

    public static final double kDrivebaseRadiusMeters = Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);

    public static final double maxRadiansPS = 0;

    public static final Translation2d[] ModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
      };
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(ModuleTranslations);

    
    public static final SwerveModuleHardwareConfig frontLeft = new SwerveModuleHardwareConfig("FrontLeft", 0, 0, 0, Rotation2d.fromRotations(0));
    public static final SwerveModuleHardwareConfig frontRight = new SwerveModuleHardwareConfig("FrontRight", 0, 0, 0, Rotation2d.fromRotations(0));
    public static final SwerveModuleHardwareConfig backLeft = new SwerveModuleHardwareConfig("BackLeft", 0, 0, 0, Rotation2d.fromRotations(0));
    public static final SwerveModuleHardwareConfig backRight = new SwerveModuleHardwareConfig("BackRight", 0, 0, 0, Rotation2d.fromRotations(0));

    public static final SwerveModuleControllerConfig azimuthControllerConfig = new SwerveModuleControllerConfig(0, 0, 0, 0, 0, 0, 0);
    public static final SwerveModuleControllerConfig driveControllerConfig = new SwerveModuleControllerConfig(0, 0, 0, 0, 0, 0, 0);

    public static final int gyroPort = 0;

    public static final double maxLinearSpped = 0;
    public static final double maxRotationalSpeed = 0;

    public static final double azimuthGearing = 0;
    public static final double driveGearing = 0;

    public static record SwerveModuleHardwareConfig(String name, int drivePort, int azimuthPort, int cancoderPort, Rotation2d offset) {}

    public static record SwerveModuleControllerConfig(int kP, int kI, int kD, int kS, int kV, int kA, int kG) {}
    
}