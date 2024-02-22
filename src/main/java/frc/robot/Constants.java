// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
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

    public static final int intakeID = 5;
    public static final int shooterID1 = 100;
    public static final int shooterID2 = 100;

    // colin shit (do not look)

    public static final double PERIOD = .02;

    // TODO: change to tunable numbers
    public static final double LIENAR_SPEED_EXPONENT = 2;
    public static final double ANGULAR_SPEED_EXPONENT = 2.5;

    public static final double ANALOG_DEADZONE = .05;

    // TODO: Should this be its own file?
    public static class HardwareConstants {
        // the port for the xbox controller
        public static final int XBOX_CONTROLLER_PORT = 0;
        public static final int XBOX_CONTROLLER_ALTERNATE_PORT = 1;

        // for mecanum drive FR = front right, FL = front left, RR = rear right, RL =
        // rear left
        public static final int FR_TALON_ID = 1;
        public static final int FL_TALON_ID = 2;
        public static final int RR_TALON_ID = 3;
        public static final int RL_TALON_ID = 4;

        public static final int TALON_FX_CPR = 2048;
        // gear ratio of the motor gearbox
        public static final double FALCON_GEARBOX_RATIO = 10.71;

        public static final double MECANUM_WHEEL_DIAMETER = 0.1524; // meters

        public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
        public static final SPI.Port ANALOG_GYRO_PORT = SPI.Port.kOnboardCS0;
        // public static final int ANALOG_GYRO_PORT = 0;

        // Robot's kinematics --> cartesian location of each wheel to the physical
        // center of the robot in meters
        public static final double WHEEL_ABSOLUTE_X_METERS = 0.2794;
        public static final double WHEEL_ABSOLUTE_Y_METERS = 0.31115;
        

        public static final Translation2d FL_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS,
                WHEEL_ABSOLUTE_Y_METERS);
        public static final Translation2d FR_WHEEL_POS = new Translation2d(WHEEL_ABSOLUTE_X_METERS,
                -WHEEL_ABSOLUTE_Y_METERS);
        public static final Translation2d RL_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS,
                WHEEL_ABSOLUTE_Y_METERS);
        public static final Translation2d RR_WHEEL_POS = new Translation2d(-WHEEL_ABSOLUTE_X_METERS,
                -WHEEL_ABSOLUTE_Y_METERS);

        public static final MecanumDriveKinematics MECANUM_KINEMATICS = new MecanumDriveKinematics(FL_WHEEL_POS,
                FR_WHEEL_POS, RL_WHEEL_POS, RR_WHEEL_POS);
    }

    public static class AutoPathConstants {
        // i feel like there has to be a better way to do this than how i did lol
        
        public final Map<Pair<Double, Command>, String> eventMarkerMap = new HashMap<>();
        
        // file names for paths
        public static final String THREE_NOTE_WING = "SThreeNote";
        public static final String FOUR_NOTE_WING = "SFourNote";
        public static final String TWO_NOTE_LEFT = "STwoNoteLeft";
        public static final String TWO_NOTE_CENTER = "STwoNoteCenter";
        public static final String TWO_NOTE_RIGHT = "STwoNoteRight";
        public static final String FOUR_NOTE_LEFT= "SFourNoteLeft";
 
        public AutoPathConstants(Intake intake) { //shooter, drive.], etc??
            //probably a way to consolidate times here cause stuff happens at the same time for similar patterns ill do it later
            eventMarkerMap.put(Pair.of(0.38, intake.startEndCommmand()), THREE_NOTE_WING);
            eventMarkerMap.put(Pair.of(2.6, intake.startEndCommmand()), THREE_NOTE_WING);

            eventMarkerMap.put(Pair.of(1.0, intake.startEndCommmand()), FOUR_NOTE_WING);
            eventMarkerMap.put(Pair.of(2.68, intake.startEndCommmand()), FOUR_NOTE_WING);
            eventMarkerMap.put(Pair.of(4.8, intake.startEndCommmand()), FOUR_NOTE_WING);

            eventMarkerMap.put(Pair.of(.91, intake.startEndCommmand()), TWO_NOTE_LEFT);
            
            eventMarkerMap.put(Pair.of(.38, intake.startEndCommmand()), TWO_NOTE_CENTER);
           
            eventMarkerMap.put(Pair.of(1.25, intake.startEndCommmand()), TWO_NOTE_RIGHT);

            eventMarkerMap.put(Pair.of(.38, intake.startEndCommmand()), FOUR_NOTE_LEFT);
            eventMarkerMap.put(Pair.of(2.31, intake.startEndCommmand()), FOUR_NOTE_LEFT);
            eventMarkerMap.put(Pair.of(5.41, intake.startEndCommmand()), FOUR_NOTE_LEFT);
        }
    }

    public static final Pose3d[] APRIL_TAG_LOCATIONS = {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 1, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 3, 0, new Rotation3d(0, 0, 0)),
    };
}
