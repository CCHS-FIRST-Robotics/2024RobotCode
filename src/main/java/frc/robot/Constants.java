// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.*;
import java.util.*;

public final class Constants {
    public static enum Mode {
        REAL,
        SIM,
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
            ArmPosition.LOWER, Degrees.of(-15),
            ArmPosition.STAGE, Degrees.of(25.5),
            ArmPosition.CLOSE_SUB, Degrees.of(18),
            ArmPosition.TEST, Degrees.of(20)
        )
    );

    public static final Pose2d SPEAKER_POSE = new Pose2d(0, 5.55, new Rotation2d(0));

    public static final Measure<Velocity<Angle>> SHOOT_SPEED_LEFT = RotationsPerSecond.of(75);
    public static final Measure<Velocity<Angle>> SHOOT_SPEED_RIGHT = RotationsPerSecond.of(95);
    public static final Measure<Velocity<Angle>> SHOOT_SPEED_AMP = RotationsPerSecond.of(50);

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
    public static final double PERIOD = 0.02;
    public static final double ANALOG_DEADZONE = 0.1;
    // ! find somewhere else for this
    public static final Pose3d[] APRIL_TAG_LOCATIONS = {
        new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
        new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)),
        new Pose3d(0, 1, 0, new Rotation3d(0, 0, 0)),
        new Pose3d(0, 3, 0, new Rotation3d(0, 0, 0)),
    };

    public static class AutoConstants {        
        public static final Measure<Voltage> INTAKE_VOLTS = Volts.of(9);
        public static final Measure<Voltage> HANDOFF_IN_VOLTS = Volts.of(4);
        public static final Measure<Voltage> HANDOFF_OUT_VOLTS = Volts.of(12);

        public static final double INIT_MOVEMENTS_TIME = 0;
        public static final double MAX_ARM_MOVE_TIME = 1.2;
        public static final double INTAKE_TIME = 1.5;
        public static final double SHOOT_TIME = 0.5;

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