// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.*;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = ROBOT_MODE.REAL;
    
    // controllers
    public static final int CONTROLLER_PORT_1 = 0;
    public static final int CONTROLLER_PORT_2 = 1;
    public static final double JOYSTICK_DEADZONE = 0.1;
    public static final double LINEAR_SPEED_EXPONENT = 2;
    public static final double ANGULAR_SPEED_EXPONENT = 2;
    
    public static class AutoConstants {
        // times
        public static final double INIT_MOVEMENTS_TIME = 0;

        // path data
        public static final String TWO_STRAIGHT_1 = "2Straight.1";
        public static final String TWO_STRAIGHT_2 = "2Straight.2";

        public static final ArrayList<String> twoStraight = new ArrayList<String>();

        static {
            twoStraight.add(TWO_STRAIGHT_1);
            twoStraight.add(TWO_STRAIGHT_2);
        }
    }
}