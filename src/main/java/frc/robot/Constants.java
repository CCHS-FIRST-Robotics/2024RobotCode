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

    public static final Mode CURRENT_MODE = Mode.REAL;

    public static enum ArmPosition {
        INTAKE,
        AMP,
        SPEAKER,
        SHOOT,
    }

    public static HashMap<ArmPosition, Measure<Angle>> ARM_POSITIONS = new HashMap<ArmPosition, Measure<Angle>>(
            Map.of(
                    ArmPosition.INTAKE, Degrees.of(-2),
                    ArmPosition.AMP, Degrees.of(95),
                    ArmPosition.SPEAKER, Degrees.of(40))); // ! TODO: actually make this accurate because it ain't

    public static final Pose2d SPEAKER_POSE = new Pose2d(0, 5.55, new Rotation2d(0));

    // ports
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int XBOX_CONTROLLER_ALTERNATE_PORT = 1;

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

    public static enum EventCommand {
        INTAKE,
        SHOOT,
        INTAKE_HANDOFF,
        SHOOTER_HANDOFF,
        DRIVE_PATH,
        ARM_SHOOT,
        ARM_HANDOFF
    }

    public static class AutoPathConstants {
        // i feel like there has to be a better way to do this than how i did lol

        public static final Map<Pair<Double, Constants.EventCommand>, ArrayList<String>> eventMarkerMap = new HashMap<>();

        // file names for paths
        public static final String THREE_NOTE_WING1 = "SThreeNote.1";
        public static final String THREE_NOTE_WING2 = "SThreeNote.2";
        public static final String THREE_NOTE_WING = "SThreeNote";
        public static final String FOUR_NOTE_WING = "SFourNote";
        public static final String FOUR_NOTE_WING1 = "SFourNote.1";
        public static final String FOUR_NOTE_WING2 = "SFourNote.2";
        public static final String FOUR_NOTE_WING3 = "SFourNote.3";
        public static final String TWO_NOTE_LEFT = "STwoNoteLeft";
        public static final String TWO_NOTE_CENTER = "STwoNoteCenter";
        public static final String TWO_NOTE_RIGHT = "STwoNoteRight";
        public static final String FOUR_NOTE_LEFT = "SFourNoteLeft";
        public static final String TWO_NOTE_TEST = "STest1.1";

        public static final ArrayList<String> threeNoteWing = new ArrayList<String>();
        public static final ArrayList<String> twoNoteTest = new ArrayList<String>();
        public static final ArrayList<String> threeNoteWingSplits = new ArrayList<String>();
        public static final ArrayList<String> fourNoteWingSplits = new ArrayList<String>();

        public static final ArrayList<ArrayList<String>> pathLists = new ArrayList<ArrayList<String>>();

        public static final double INTAKE_VOLTS = 12;
        public static final double SHOOT_VOLTS = 12;
        public static final double INTAKE_HANDOFF_VOLTS = 5;
        public static final double SHOOTER_HANDOFF_VOLTS = 6;
        public static final Measure<Angle> ARM_HANDOFF_ANGLE = Radians.of(80 * Math.PI / 180); // double check
                                                                                               // for final
        public static final Measure<Angle> QUOKKA_ARM_INTAKE_ANGLE = Radians.of(80 * Math.PI / 180); // double
                                                                                                     // check

        public static final double INIT_MOVEMENTS_TIME = 0.0;
        public static final double MAX_ARM_MOVE_TIME = 0.5;
        public static final double INTAKE_TIME = 0.3;
        public static final double SHOOT_TIME = 0.5;// idk

        // quokka auto consts
        public static final double Q_MAX_ARM_MOVE_TIME = .5;
        public static final double Q_INTAKE_SET_TIME = 0.4;
        public static final double Q_INTAKE_TIME = 0.1;
        public static final double Q_INIT_SHOOT_SET_TIME = 2;
        public static final double Q_SHOOT_SET_TIME = 1;
        public static final double Q_SHOOT_TIME = 0.4;
        public static final Measure<Velocity<Angle>> SHOOT_SPEED = RotationsPerSecond.of(95); // idk

        static {
            threeNoteWing.add(THREE_NOTE_WING);
            threeNoteWing.add(THREE_NOTE_WING1);
            threeNoteWing.add(THREE_NOTE_WING2);

            threeNoteWingSplits.add(THREE_NOTE_WING1);
            threeNoteWingSplits.add(THREE_NOTE_WING2);

            fourNoteWingSplits.add(FOUR_NOTE_WING1);
            fourNoteWingSplits.add(FOUR_NOTE_WING2);
            fourNoteWingSplits.add(FOUR_NOTE_WING3);

            twoNoteTest.add(TWO_NOTE_TEST);

            pathLists.add(threeNoteWing);

            /*
             * three note w/ quokka; 1 drive traj
             */
            eventMarkerMap.put(Pair.of(0.0, EventCommand.DRIVE_PATH), threeNoteWing);
            eventMarkerMap.put(Pair.of(0.0, EventCommand.ARM_SHOOT), threeNoteWing);
            eventMarkerMap.put(Pair.of(0.15, EventCommand.SHOOT), threeNoteWing);
            eventMarkerMap.put(Pair.of(0.25, EventCommand.ARM_HANDOFF), threeNoteWing);
            eventMarkerMap.put(Pair.of(0.36, EventCommand.INTAKE), threeNoteWing);
            eventMarkerMap.put(Pair.of(0.74, EventCommand.ARM_SHOOT), threeNoteWing);
            eventMarkerMap.put(Pair.of(0.9, EventCommand.SHOOT), threeNoteWing);
            eventMarkerMap.put(Pair.of(1.1, EventCommand.ARM_HANDOFF), threeNoteWing);
            eventMarkerMap.put(Pair.of(1.2, EventCommand.INTAKE), threeNoteWing);
            eventMarkerMap.put(Pair.of(1.5, EventCommand.ARM_SHOOT), threeNoteWing);
            eventMarkerMap.put(Pair.of(1.58, EventCommand.SHOOT), threeNoteWing);

            // probably a way to consolidate times here cause stuff happens at the same time
            // for similar patterns ill do it later
            /*
             * commands that work for a final robot intake
             */
            // eventMarkerMap.put(Pair.of(0.0, EventCommand.ARM_SHOOT), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.0, EventCommand.SHOOT), threeNoteWing); //
            // change
            // eventMarkerMap.put(Pair.of(0.0, EventCommand.DRIVE_PATH), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.3, EventCommand.ARM_HANDOFF), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.4, EventCommand.INTAKE), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.6, EventCommand.INTAKE_HANDOFF), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.6, EventCommand.SHOOTER_HANDOFF),
            // threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.67, EventCommand.ARM_SHOOT), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.77, EventCommand.SHOOT), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.77, EventCommand.DRIVE_PATH), threeNoteWing);
            // eventMarkerMap.put(Pair.of(0.8, EventCommand.ARM_HANDOFF), threeNoteWing);
            // eventMarkerMap.put(Pair.of(1.31, EventCommand.INTAKE), threeNoteWing);
            // eventMarkerMap.put(Pair.of(1.5, EventCommand.INTAKE_HANDOFF), threeNoteWing);
            // eventMarkerMap.put(Pair.of(1.5, EventCommand.SHOOTER_HANDOFF),
            // threeNoteWing);
            // eventMarkerMap.put(Pair.of(1.6, EventCommand.ARM_SHOOT), threeNoteWing);
            // eventMarkerMap.put(Pair.of(1.67, EventCommand.SHOOT), threeNoteWing);

            // eventMarkerMap.put(Pair.of(1.0, intake.startEndCommmand()), FOUR_NOTE_WING);
            // eventMarkerMap.put(Pair.of(2.68, intake.startEndCommmand()), FOUR_NOTE_WING);
            // eventMarkerMap.put(Pair.of(4.8, intake.startEndCommmand()), FOUR_NOTE_WING);

            // eventMarkerMap.put(Pair.of(.91, intake.startEndCommmand()), TWO_NOTE_LEFT);

            // eventMarkerMap.put(Pair.of(.38, intake.startEndCommmand()), TWO_NOTE_CENTER);

            // eventMarkerMap.put(Pair.of(1.25, intake.startEndCommmand()), TWO_NOTE_RIGHT);

            // eventMarkerMap.put(Pair.of(.38, intake.startEndCommmand()), FOUR_NOTE_LEFT);
            // eventMarkerMap.put(Pair.of(2.31, intake.startEndCommmand()), FOUR_NOTE_LEFT);
            // eventMarkerMap.put(Pair.of(5.41, intake.startEndCommmand()), FOUR_NOTE_LEFT);

            /*
             * start against speaker (1.4, 5.55, 0)
             * WM (2.75, 5.55, 0)
             * WM-WL angle .71
             * WR
             * 
             * 
             */

        }
    }
}