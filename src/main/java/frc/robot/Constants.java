// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import java.util.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Mode CURRENT_MODE = Mode.SIM;

    public static enum ArmPosition {
        INTAKE,
        AMP,
        SPEAKER,
        SHOOT,
        MAIN,
        LOWER,
        SHOOT_MID,
        SHOOT_FAR
    }

    public static HashMap<ArmPosition, Measure<Angle>> ARM_POSITIONS = new HashMap<ArmPosition, Measure<Angle>>(
            Map.of(
                    ArmPosition.INTAKE, Degrees.of(-20),
                    ArmPosition.AMP, Degrees.of(88),
                    ArmPosition.SPEAKER, Degrees.of(5),
                    ArmPosition.MAIN, Degrees.of(-23),
                    ArmPosition.LOWER, Degrees.of(-10),
                    ArmPosition.SHOOT_MID, Degrees.of(20),
                    ArmPosition.SHOOT_FAR, Degrees.of(35))); // ! TODO: actually make this accurate because it ain't

    public static final Pose2d SPEAKER_POSE = new Pose2d(0, 5.55, new Rotation2d(0));

    public static final Measure<Velocity<Angle>> SHOOTER_LEFT_SPEED = RotationsPerSecond.of(95);
    public static final Measure<Velocity<Angle>> SHOOTER_RIGHT_SPEED = RotationsPerSecond.of(95);

    public static final Measure<Velocity<Angle>> SHOOTER_AMP_SPEED = RotationsPerSecond.of(95);

    // ports
    public static final int CONTROLLER_PORT_1 = 0;
    public static final int CONTROLLER_PORT_2 = 1;

    // ids
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

    // TODO: change to tunable numbers
    public static final double LIENAR_SPEED_EXPONENT = 1.8;
    public static final double ANGULAR_SPEED_EXPONENT = 2;

    // private static final Measure SPEAKER_SHOOT_ANGLE = new Measure.

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

        public static final Map<Pair<Double, Constants.EventCommand>, ArrayList<String>> eventMarkerMap = new HashMap<>();

        // file names for paths
        public static final String TWO_NOTE_LEFT = "TwoNoteLeftSS.1";
        public static final String TWO_NOTE_CENTER = "TwoNoteCenterSS.1";
        public static final String TWO_NOTE_RIGHT = "TwoNoteRightSS.1";

        public static final String THREE_NOTE_WRR1 = "ThreeNoteRightRSS.1";
        public static final String THREE_NOTE_WRR2 = "ThreeNoteRightRSS.2";
        public static final String THREE_NOTE_WRC1 = "ThreeNoteRightCSS.1";
        public static final String THREE_NOTE_WRC2 = "ThreeNoteRightCSS.2";
        public static final String THREE_NOTE_WLL1 = "ThreeNoteLeftLSS.1";
        public static final String THREE_NOTE_WLL2 = "ThreeNoteLeftLSS.2";
        public static final String THREE_NOTE_WLC1 = "ThreeNoteLeftCSS.1";
        public static final String THREE_NOTE_WLC2 = "ThreeNoteLeftCSS.2";

        public static final String FOUR_NOTE_WING1 = "FourNoteWingSS.1";
        public static final String FOUR_NOTE_WING2 = "FourNoteWingSS.2";
        public static final String FOUR_NOTE_WING3 = "FourNoteWingSS.3";
        public static final String FOUR_NOTE_L1 = "FourNoteLeftSS.1";
        public static final String FOUR_NOTE_L2 = "FourNoteLeftSS.2";
        public static final String FOUR_NOTE_L3 = "FourNoteLeftSS.3";

        public static final String TESTING_RED1 = "TestingSS.1";

        public static final String TWOCL = "2CL.1";

        public static final String B_THREE_LCLSS1 = "BlueThreeLCLSS.1";
        public static final String B_THREE_LCLSS2 = "BlueThreeLCLSS.2";

        public static final String B_THREE_RCLSS1 = "BlueThreeRCLSS.1";
        public static final String B_THREE_RCLSS2 = "BlueThreeRCLSS.2";

        public static final String B_THREE_2CL1 = "BlueThreeNoteCLSS.1";
        public static final String B_THREE_2CL2 = "BlueThreeNoteCLSS.2";

        public static final String R_THREE_CLSS1 = "RedThreeNoteCLSS.1";
        public static final String R_THREE_CLSS2 = "RedThreeNoteCLSS.2";

        public static final ArrayList<String> twoNoteLeft = new ArrayList<String>();
        public static final ArrayList<String> twoNoteRight = new ArrayList<String>();
        public static final ArrayList<String> twoNoteCenter = new ArrayList<String>();
        public static final ArrayList<String> threeNoteWRR = new ArrayList<String>();
        public static final ArrayList<String> threeNoteWRC = new ArrayList<String>();
        public static final ArrayList<String> threeNoteWLL = new ArrayList<String>();
        public static final ArrayList<String> threeNoteWLC = new ArrayList<String>();
        public static final ArrayList<String> fourNoteWing = new ArrayList<String>();
        public static final ArrayList<String> fourNoteLeft = new ArrayList<String>();

        public static final ArrayList<String> blueThreeLCLSS = new ArrayList<String>();
        public static final ArrayList<String> blueThreeRCLSS = new ArrayList<String>();
        public static final ArrayList<String> blueThree2CLSS = new ArrayList<String>();
        public static final ArrayList<String> redThreeCLSS = new ArrayList<String>();

        public static final ArrayList<String> twoCLpath = new ArrayList<String>();

        public static final ArrayList<String> testingRed = new ArrayList<String>();

        public static final Measure<Voltage> INTAKE_VOLTS = Volts.of(4);
        public static final Measure<Voltage> HANDOFF_IN_VOLTS = Volts.of(6);
        public static final Measure<Voltage> HANDOFF_OUT_VOLTS = Volts.of(12);

        public static final double INIT_MOVEMENTS_TIME = 0.0;
        public static final double MAX_ARM_MOVE_TIME = .5;
        public static final double INTAKE_TIME = 0.3;
        public static final double SHOOT_TIME = 0.5;

        public static final Pose2d AUTO_START_POS = new Pose2d(1.3, 5.544, new Rotation2d(0));

        public static final Measure<Velocity<Angle>> SHOOT_SPEED_LEFT = RotationsPerSecond.of(95);
        public static final Measure<Velocity<Angle>> SHOOT_SPEED_RIGHT = RotationsPerSecond.of(95); // idk

        static {
            twoNoteLeft.add(TWO_NOTE_LEFT);
            twoNoteCenter.add(TWO_NOTE_CENTER);
            twoNoteRight.add(TWO_NOTE_RIGHT);

            threeNoteWRR.add(THREE_NOTE_WRR1);
            threeNoteWRR.add(THREE_NOTE_WRR2);

            threeNoteWRC.add(THREE_NOTE_WRC1);
            threeNoteWRC.add(THREE_NOTE_WRC2);

            threeNoteWLL.add(THREE_NOTE_WLL1);
            threeNoteWLL.add(THREE_NOTE_WLL2);

            threeNoteWLC.add(THREE_NOTE_WLC1);
            threeNoteWLC.add(THREE_NOTE_WLC2);

            fourNoteLeft.add(FOUR_NOTE_L1);
            fourNoteLeft.add(FOUR_NOTE_L2);
            fourNoteLeft.add(FOUR_NOTE_L3);

            fourNoteWing.add(FOUR_NOTE_WING1);
            fourNoteWing.add(FOUR_NOTE_WING2);
            fourNoteWing.add(FOUR_NOTE_WING3);

            twoCLpath.add(TWOCL);

            testingRed.add(TESTING_RED1);

            blueThree2CLSS.add(B_THREE_2CL1);
            blueThree2CLSS.add(B_THREE_2CL2);

            blueThreeLCLSS.add(B_THREE_LCLSS1);
            blueThreeLCLSS.add(B_THREE_LCLSS2);

            blueThreeRCLSS.add(B_THREE_RCLSS1);
            blueThreeRCLSS.add(B_THREE_RCLSS2);

            redThreeCLSS.add(R_THREE_CLSS1);
            redThreeCLSS.add(R_THREE_CLSS2);
        }
    }
}