// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;

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
}