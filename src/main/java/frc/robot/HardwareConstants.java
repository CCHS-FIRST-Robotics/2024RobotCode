package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;

public class HardwareConstants {
    // kinematics
    // ! units chat units
    public static final double WHEEL_ABSOLUTE_X_METERS = 0.2794;
    public static final double WHEEL_ABSOLUTE_Y_METERS = 0.31115;
    public static final int TALON_FX_CPR = 2048;
    public static final double FALCON_GEARBOX_RATIO = 10.71;

    public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
    public static final Translation2d FR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);
    public static final Translation2d BL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, WHEEL_ABSOLUTE_Y_METERS);
    public static final Translation2d BR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS, -WHEEL_ABSOLUTE_Y_METERS);

    // swerve constants
    public static final double LINEAR_SPEED_EXPONENT = 2;
    public static final double ANGULAR_SPEED_EXPONENT = 2;

    public static final Measure<Distance> trackWidthX = Inches.of(22.5);
    public static final Measure<Distance> trackWidthY = Inches.of(22.5);
    public static final Measure<Velocity<Distance>> maxLinearSpeed = MetersPerSecond.of(4.5);
    public static final Measure<Velocity<Velocity<Distance>>> maxLinearAcceleration = MetersPerSecondPerSecond.of(9.0);
    public static final Measure<Velocity<Angle>> maxAngularSpeed = RadiansPerSecond.of(8 * Math.PI);
    public static final Measure<Velocity<Velocity<Angle>>> maxAngularAcceleration = RadiansPerSecond.per(Seconds).of(10 * Math.PI);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(-trackWidthX.in(Meters) / 2.0, -trackWidthY.in(Meters) / 2.0),
        new Translation2d(trackWidthX.in(Meters) / 2.0, -trackWidthY.in(Meters) / 2.0),
        new Translation2d(trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0),
        new Translation2d(-trackWidthX.in(Meters) / 2.0, trackWidthY.in(Meters) / 2.0)
    };

    // motor constants
    public static final int FALCON_MAX_RPS = 7500 / 60;
    public static final int CIM_MAX_RPS = 5330 / 60;
    public static final int NEO_MAX_RPS = 5676 / 60;

    // arm constants
    public static final Measure<Distance> ARM_LENGTH = Inches.of(16);
    public static final Translation2d ARM_OFFSET = new Translation2d(0.0, .425);
}