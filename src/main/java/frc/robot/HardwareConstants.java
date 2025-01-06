package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.*;

public class HardwareConstants {
    // swerve stats
    public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED = MetersPerSecond.of(4.5);
    public static final Measure<Velocity<Velocity<Distance>>> MAX_LINEAR_ACCEL = MetersPerSecondPerSecond.of(9.0);
    public static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED = RadiansPerSecond.of(8 * Math.PI);
    public static final Measure<Velocity<Velocity<Angle>>> MAX_ANGULAR_ACCEL = RadiansPerSecond.per(Seconds).of(10 * Math.PI);
    
    // kinematics
    public static final Measure<Distance> TRACK_WIDTH_X = Inches.of(22.5);
    public static final Measure<Distance> TRACK_WIDTH_Y = Inches.of(22.5);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0),
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0)
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
}