// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import java.util.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Mode CURRENT_MODE = Mode.REAL;

    public static enum ArmPosition {
        INTAKE,
        AMP,
        SPEAKER,
        SHOOT,
        MAIN,
        LOWER,
        STAGE,
        CLOSE_SUB,
        TEST
    }

    public static HashMap<ArmPosition, Measure<Angle>> ARM_POSITIONS = new HashMap<ArmPosition, Measure<Angle>>(
            Map.of(
                    ArmPosition.INTAKE, Degrees.of(-8),
                    ArmPosition.AMP, Degrees.of(85),
                    ArmPosition.SPEAKER, Degrees.of(6),
                    ArmPosition.MAIN, Degrees.of(-22),
                    ArmPosition.LOWER, Degrees.of(0),
                    ArmPosition.STAGE, Degrees.of(25.5),
                    ArmPosition.CLOSE_SUB, Degrees.of(18),
                    ArmPosition.TEST, Degrees.of(20)));

    public static final Pose2d SPEAKER_POSE = new Pose2d(0, 5.55, new Rotation2d(0));

    public static final Measure<Velocity<Angle>> SHOOTER_LEFT_SPEED = RotationsPerSecond.of(75);
    public static final Measure<Velocity<Angle>> SHOOTER_RIGHT_SPEED = RotationsPerSecond.of(95);
    public static final Measure<Velocity<Angle>> SHOOTER_AMP_SPEED = RotationsPerSecond.of(50);

    // ports and ids
    public static final int CONTROLLER_PORT_1 = 0;
    public static final int CONTROLLER_PORT_2 = 1;
    public static final int ARM_CANCODER_ID = 19;
    public static final int ARM_LEAD_ID = 20;
    public static final int ARM_FOLLOW_ID = 21;
    public static final int SHOOTER_ID_1 = 22;
    public static final int SHOOTER_ID_2 = 23;
    public static final int HANDOFF_ID = 24;
    public static final int INTAKE_ID1 = 25;
    public static final int INTAKE_ID2 = 26;

    // miscellaneous
    public static final double PERIOD = .02;
    public static final double ANALOG_DEADZONE = .02;
    public static final Pose3d[] APRIL_TAG_LOCATIONS = {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 1, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 3, 0, new Rotation3d(0, 0, 0)),
    };

    // ! what're these for
    public static final double LINEAR_SPEED_EXPONENT = 3.5;
    public static final double ANGULAR_SPEED_EXPONENT = 3;

    public static enum EventCommand {
        INTAKE,
        SHOOT,
        INTAKE_HANDOFF,
        SHOOTER_HANDOFF,
        DRIVE_PATH,
        ARM_SHOOT,
        ARM_HANDOFF
    }

    public static class StartPosistions {
        public static final Pose2d blueAmp = new Pose2d(0.790, 6.550, new Rotation2d());
        public static final Pose2d blueCenter = new Pose2d(1.3, 5.544, new Rotation2d());
        public static final Pose2d blueSource = new Pose2d(0.79, 4.55, new Rotation2d());
        public static final Pose2d redAmp = new Pose2d(15.75, 6.55, new Rotation2d());
        public static final Pose2d redCenter = new Pose2d(15.27, 5.544, new Rotation2d());
        public static final Pose2d redSource = new Pose2d(15.75, 4.55, new Rotation2d());
    }

    public static class AutoPathConstants {

        public static final Alliance ALLIANCE = Alliance.Red;

        public static final Map<Pair<Double, Constants.EventCommand>, ArrayList<String>> eventMarkerMap = new HashMap<>();

        // file names for paths
        public static final String TWO_NOTE_A1 = "2A1.1";
        public static final String TWO_NOTE_C2 = "2C2.1";
        public static final String TWO_NOTE_S3 = "2S3.1";
        public static final String TWO_NOTE_S8 = "2S8.1";

        public static final String THREE_NOTE_C21_1 = "3C21.1";
        public static final String THREE_NOTE_C21_2 = "3C21.2";

        public static final String THREE_NOTE_C23_1 = "3C23.1";
        public static final String THREE_NOTE_C23_2 = "3C23.2";

        public static final String THREE_NOTE_A14_1 = "3A14.1";
        public static final String THREE_NOTE_A14_2 = "3A14.2";

        public static final String THREE_NOTE_S37_1 = "3S37.1";
        public static final String THREE_NOTE_S37_2 = "3S37.2";

        public static final String THREE_NOTE_C25_1 = "3C25.1";
        public static final String THREE_NOTE_C25_2 = "3C25.2";

        public static final String THREE_NOTE_S87_1 = "3S87.1";
        public static final String THREE_NOTE_S87_2 = "3S87.2";

        public static final String THREE_NOTE_S32_1 = "3S32.1";
        public static final String THREE_NOTE_S32_2 = "3S32.2";

        public static final String THREE_NOTE_A12_1 = "3A12.1";
        public static final String THREE_NOTE_A12_2 = "3A12.2";

        public static final String FOUR_NOTE_C231_1 = "4C231.1";
        public static final String FOUR_NOTE_C231_2 = "4C231.2";
        public static final String FOUR_NOTE_C231_3 = "4C231.3";

        public static final String FOUR_NOTE_C214_1 = "4C214.1";
        public static final String FOUR_NOTE_C214_2 = "4C214.2";
        public static final String FOUR_NOTE_C214_3 = "4C214.3";

        public static final String FOUR_NOTE_C238_1 = "4C238.1";
        public static final String FOUR_NOTE_C238_2 = "4C238.2";
        public static final String FOUR_NOTE_C238_3 = "4C238.3";

        public static final String FIVE_NOTE_C3214_1 = "5C3214.1";
        public static final String FIVE_NOTE_C3214_2 = "5C3214.2";
        public static final String FIVE_NOTE_C3214_3 = "5C3214.3";
        public static final String FIVE_NOTE_C3214_4 = "5C3214.4";

        public static final String RED_FOUR_WING_1 = "RedFourWing.1";
        public static final String RED_FOUR_WING_2 = "RedFourWing.2";
        public static final String RED_FOUR_WING_3 = "RedFourWing.3";

        public static final String RED_FOUR_4C215_1 = "Red4C215.1";
        public static final String RED_FOUR_4C215_2 = "Red4C215.2";
        public static final String RED_FOUR_4C215_3 = "Red4C215.3";

        public static final String RED_FOUR_4C237_1 = "Red4C237.1";
        public static final String RED_FOUR_4C237_2 = "Red4C237.2";
        public static final String RED_FOUR_4C237_3 = "Red4C237.3";

        public static final String RED_FOUR_4S378_1 = "Red4C378.1";
        public static final String RED_FOUR_4S378_2 = "Red4C378.2";
        public static final String RED_FOUR_4S378_3 = "Red4C378.3";

        // array lists for paths
        public static final ArrayList<String> twoC2 = new ArrayList<String>();
        public static final ArrayList<String> twoA1 = new ArrayList<String>();
        public static final ArrayList<String> twoS3 = new ArrayList<String>();
        public static final ArrayList<String> twoS8 = new ArrayList<String>();
        public static final ArrayList<String> threeC21 = new ArrayList<String>();
        public static final ArrayList<String> threeC23 = new ArrayList<String>();
        public static final ArrayList<String> threeA14 = new ArrayList<String>();
        public static final ArrayList<String> threeS37 = new ArrayList<String>();
        public static final ArrayList<String> threeC25 = new ArrayList<String>();
        public static final ArrayList<String> threeS87 = new ArrayList<String>();
        public static final ArrayList<String> threeS32 = new ArrayList<String>();
        public static final ArrayList<String> threeA12 = new ArrayList<String>();
        public static final ArrayList<String> fourC231 = new ArrayList<String>();
        public static final ArrayList<String> fourC238 = new ArrayList<String>();
        public static final ArrayList<String> fourC214 = new ArrayList<String>();
        public static final ArrayList<String> fiveC3214 = new ArrayList<String>();
        public static final ArrayList<String> redFourWing = new ArrayList<String>();
        public static final ArrayList<String> red4C215 = new ArrayList<String>();
        public static final ArrayList<String> red4C237 = new ArrayList<String>();
        public static final ArrayList<String> red4S378 = new ArrayList<String>();

        public static final Measure<Voltage> INTAKE_VOLTS = Volts.of(9);
        public static final Measure<Voltage> HANDOFF_IN_VOLTS = Volts.of(3.5);
        public static final Measure<Voltage> HANDOFF_OUT_VOLTS = Volts.of(12);

        public static final double INIT_MOVEMENTS_TIME = 0.0;
        public static final double MAX_ARM_MOVE_TIME = 1.2;
        public static final double INTAKE_TIME = .5;
        public static final double SHOOT_TIME = 0.5;

        public static final Pose2d AUTO_START_POS = new Pose2d(1.3, 5.544, new Rotation2d(0));

        public static final Measure<Velocity<Angle>> SHOOT_SPEED_LEFT = RotationsPerSecond.of(90);
        public static final Measure<Velocity<Angle>> SHOOT_SPEED_RIGHT = RotationsPerSecond.of(95); // idk

        static {
            twoA1.add(TWO_NOTE_A1);
            twoC2.add(TWO_NOTE_C2);
            twoS3.add(TWO_NOTE_S3);
            twoS8.add(TWO_NOTE_S8);

            threeC21.add(THREE_NOTE_C21_1);
            threeC21.add(THREE_NOTE_C21_2);

            threeC23.add(THREE_NOTE_C23_1);
            threeC23.add(THREE_NOTE_C23_2);

            threeA14.add(THREE_NOTE_A14_1);
            threeA14.add(THREE_NOTE_A14_2);

            threeS37.add(THREE_NOTE_S37_1);
            threeS37.add(THREE_NOTE_S37_2);

            threeC25.add(THREE_NOTE_C25_1);
            threeC25.add(THREE_NOTE_C25_2);

            threeS87.add(THREE_NOTE_S87_1);
            threeS87.add(THREE_NOTE_S87_2);

            threeS32.add(THREE_NOTE_S32_1);
            threeS32.add(THREE_NOTE_S32_2);

            threeA12.add(THREE_NOTE_A12_1);
            threeA12.add(THREE_NOTE_A12_2);

            fourC231.add(FOUR_NOTE_C231_1);
            fourC231.add(FOUR_NOTE_C231_2);
            fourC231.add(FOUR_NOTE_C231_3);

            fourC238.add(FOUR_NOTE_C238_1);
            fourC238.add(FOUR_NOTE_C238_2);
            fourC238.add(FOUR_NOTE_C238_3);

            fourC214.add(FOUR_NOTE_C214_1);
            fourC214.add(FOUR_NOTE_C214_2);
            fourC214.add(FOUR_NOTE_C214_3);

            fiveC3214.add(FIVE_NOTE_C3214_1);
            fiveC3214.add(FIVE_NOTE_C3214_2);
            fiveC3214.add(FIVE_NOTE_C3214_3);
            fiveC3214.add(FIVE_NOTE_C3214_4);

            redFourWing.add(RED_FOUR_WING_1);
            redFourWing.add(RED_FOUR_WING_2);
            redFourWing.add(RED_FOUR_WING_3);

            red4C215.add(RED_FOUR_4C215_1);
            red4C215.add(RED_FOUR_4C215_2);
            red4C215.add(RED_FOUR_4C215_3);

            red4C237.add(RED_FOUR_4C237_1);
            red4C237.add(RED_FOUR_4C237_2);
            red4C237.add(RED_FOUR_4C237_3);

            red4S378.add(RED_FOUR_4S378_1);
            red4S378.add(RED_FOUR_4S378_2);
            red4S378.add(RED_FOUR_4S378_3);
        }

        /*
         * AUTO PATHS
         * starting pose: Amp(A), Center(C), Source(S)
         */
    }
}