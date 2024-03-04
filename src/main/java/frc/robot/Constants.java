// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import edu.wpi.first.math.geometry.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Constants {
        public static final Mode currentMode = Mode.SIM;

        public static enum Mode {
                /** Running on a real robot. */
                REAL,

                /** Running a physics simulator. */
                SIM,

                /** Replaying from a log file. */
                REPLAY
        }

    public static final Mode CURRENT_MODE = Mode.SIM;

    // ports
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int XBOX_CONTROLLER_ALTERNATE_PORT = 1;

    // ids
    public static final int FL_TALON_ID = 1;
    public static final int FR_TALON_ID = 2;
    public static final int BL_TALON_ID = 3;
    public static final int BR_TALON_ID = 4;
    public static final int INTAKE_ID = 21;
    public static final int SHOOTER_ID_1 = 22;
    public static final int SHOOTER_ID_2 = 23;

    // miscellaneous
    public static final double PERIOD = .02;
    public static final double ANALOG_DEADZONE = .05;
    public static final Pose3d[] APRIL_TAG_LOCATIONS = {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 1, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 3, 0, new Rotation3d(0, 0, 0)),
    };

        // TODO: change to tunable numbers
        public static final double LIENAR_SPEED_EXPONENT = 2;
        public static final double ANGULAR_SPEED_EXPONENT = 2.5;


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
                public static final String TWO_NOTE_LEFT = "STwoNoteLeft";
                public static final String TWO_NOTE_CENTER = "STwoNoteCenter";
                public static final String TWO_NOTE_RIGHT = "STwoNoteRight";
                public static final String FOUR_NOTE_LEFT = "SFourNoteLeft";

                public static final ArrayList<String> threeNoteWing = new ArrayList<String>();
                public static final ArrayList<String> threeNoteWingSplits = new ArrayList<String>();

                public static final ArrayList<ArrayList<String>> pathLists = new ArrayList<ArrayList<String>>();

                public static final double INTAKE_VOLTS = 12;
                public static final double SHOOT_VOLTS = 12;
                public static final double INTAKE_HANDOFF_VOLTS = 6;
                public static final double SHOOTER_HANDOFF_VOLTS = 6;
                public static final Measure<Angle> ARM_HANDOFF_ANGLE = Radians.of(80 * Math.PI / 180); // double check for final
                public static final Measure<Angle> QUOKKA_ARM_INTAKE_ANGLE = Radians.of(80 * Math.PI / 180); // double check

                public static final double INIT_MOVEMENTS_TIME = 0.0;
                public static final double MAX_ARM_MOVE_TIME = 0.5;
                public static final double INTAKE_TIME = 0.3;

                public static final int DRIVE_POS = 0;
                public static final int ARM_SHOOT = 1;
                public static final int SHOOT = 2;
                public static final int ARM_INTAKE = 3;
                public static final int INTAKE = 4;


                static {
                        threeNoteWing.add(THREE_NOTE_WING);
                        threeNoteWing.add(THREE_NOTE_WING1);
                        threeNoteWing.add(THREE_NOTE_WING2);

                        threeNoteWingSplits.add(THREE_NOTE_WING1);
                        threeNoteWingSplits.add(THREE_NOTE_WING2);

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
                        // eventMarkerMap.put(Pair.of(0.0, EventCommand.SHOOT), threeNoteWing); // change
                        // eventMarkerMap.put(Pair.of(0.0, EventCommand.DRIVE_PATH), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.3, EventCommand.ARM_HANDOFF), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.4, EventCommand.INTAKE), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.6, EventCommand.INTAKE_HANDOFF), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.6, EventCommand.SHOOTER_HANDOFF), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.67, EventCommand.ARM_SHOOT), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.77, EventCommand.SHOOT), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.77, EventCommand.DRIVE_PATH), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(0.8, EventCommand.ARM_HANDOFF), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(1.31, EventCommand.INTAKE), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(1.5, EventCommand.INTAKE_HANDOFF), threeNoteWing);
                        // eventMarkerMap.put(Pair.of(1.5, EventCommand.SHOOTER_HANDOFF), threeNoteWing);
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