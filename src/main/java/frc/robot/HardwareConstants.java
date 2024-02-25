package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class HardwareConstants {
        // kinematics
        public static final double WHEEL_ABSOLUTE_X_METERS = 0.2794;
        public static final double WHEEL_ABSOLUTE_Y_METERS = 0.31115;
        public static final double MECANUM_WHEEL_DIAMETER = 0.1524; // meters
        public static final int TALON_FX_CPR = 2048;
        public static final double FALCON_GEARBOX_RATIO = 10.71;

        public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS,
                        WHEEL_ABSOLUTE_Y_METERS);
        public static final Translation2d FR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS,
                        -WHEEL_ABSOLUTE_Y_METERS);
        public static final Translation2d BL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS,
                        WHEEL_ABSOLUTE_Y_METERS);
        public static final Translation2d BR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS,
                        -WHEEL_ABSOLUTE_Y_METERS);

        public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(FL_WHEEL_POS,
                        FR_WHEEL_POS, BL_WHEEL_POS, BR_WHEEL_POS);

        // swerve constants
        // TODO: change to tunable numbers
        public static final double LINEAR_SPEED_EXPONENT = 2;
        public static final double ANGULAR_SPEED_EXPONENT = 2.5;

        // motor constants
        public static final int FALCON_MAX_RPM = 7500;
        public static final int CIM_MAX_RPM = 5330;
        public static final int NEO_MAX_RPM = 5676;
}