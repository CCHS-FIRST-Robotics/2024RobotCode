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
        MAIN,
        SHOOT_MID,
        SHOOT_FAR
    }

    public static HashMap<ArmPosition, Measure<Angle>> ARM_POSITIONS = new HashMap<ArmPosition, Measure<Angle>>(
            Map.of(
                    ArmPosition.INTAKE, Degrees.of(-14),
                    ArmPosition.AMP, Degrees.of(95),
                    ArmPosition.SPEAKER, Degrees.of(4),
                    ArmPosition.MAIN, Degrees.of(-5),
                    ArmPosition.SHOOT_MID, Degrees.of(20),
                    ArmPosition.SHOOT_FAR, Degrees.of(35)
            )
    ); // ! TODO: actually make this accurate because it ain't

    public static final Pose2d SPEAKER_POSE = new Pose2d(0, 5.55, new Rotation2d(0));

    public static final Measure<Velocity<Angle>> SHOOTER_LEFT_SPEED = RotationsPerSecond.of(95);
    public static final Measure<Velocity<Angle>> SHOOTER_RIGHT_SPEED = RotationsPerSecond.of(95);

    public static final Measure<Velocity<Angle>> SHOOTER_AMP_SPEED = RotationsPerSecond.of(30);

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

    public static class AutoPathConstants {
        // all auto paths starts at (1.3, 5.544)

        public static final Map<Pair<Double, Constants.EventCommand>, ArrayList<String>> eventMarkerMap = new HashMap<>();

        // file names for paths
        public static final String THREE_NOTE_WING1 = "ThreeNoteWing.1";
        public static final String THREE_NOTE_WING2 = "ThreeNoteWing.2";
        public static final String FOUR_NOTE_WING1 = "FourNoteWing.1";
        public static final String FOUR_NOTE_WING2 = "FourNoteWing.2";
        public static final String FOUR_NOTE_WING3 = "FourNoteWing.3";
        public static final String TWO_NOTE_LEFT = "TwoNoteLeft.1";
        public static final String TWO_NOTE_CENTER = "TwoNoteCenter.1";
        public static final String TWO_NOTE_RIGHT = "TwoNoteRight.1";

        public static final ArrayList<String> twoNoteTest = new ArrayList<String>();
        public static final ArrayList<String> threeNoteWingSplits = new ArrayList<String>();
        public static final ArrayList<String> fourNoteWingSplits = new ArrayList<String>();

        public static final double INTAKE_VOLTS = 6;
        public static final double HANDOFF_VOLTS = 6;
  
        public static final double INIT_MOVEMENTS_TIME = 0.0;
        public static final double MAX_ARM_MOVE_TIME = .5;
        public static final double INTAKE_TIME = 0.1;
        public static final double SHOOT_TIME = 0.4;

        public static final Pose2d AUTO_START_POS = new Pose2d(1.3, 5.544, new Rotation2d(0));

        public static final Measure<Velocity<Angle>> SHOOT_SPEED_LEFT = RotationsPerSecond.of(95);
        public static final Measure<Velocity<Angle>> SHOOT_SPEED_RIGHT = RotationsPerSecond.of(95); // idk

        static {
            threeNoteWingSplits.add(THREE_NOTE_WING1);
            threeNoteWingSplits.add(THREE_NOTE_WING2);

            fourNoteWingSplits.add(FOUR_NOTE_WING1);
            fourNoteWingSplits.add(FOUR_NOTE_WING2);
            fourNoteWingSplits.add(FOUR_NOTE_WING3);

            twoNoteTest.add(TWO_NOTE_CENTER);
        }
    }
}