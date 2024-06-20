// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;

public final class Constants {
    // ports and ids
    public static final int CONTROLLER_PORT = 0;

    // miscellaneous
    public static final double PERIOD = .02;
    public static final double ANALOG_DEADZONE = .02;
    public static final Pose3d[] APRIL_TAG_LOCATIONS = {
            new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 1, 0, new Rotation3d(0, 0, 0)),
            new Pose3d(0, 3, 0, new Rotation3d(0, 0, 0)),
    };

    public static class StartPosistions {
        public static final Pose2d blueAmp = new Pose2d(0.790, 6.550, new Rotation2d());
        public static final Pose2d blueCenter = new Pose2d(1.3, 5.544, new Rotation2d());
        public static final Pose2d blueSource = new Pose2d(0.79, 4.55, new Rotation2d());
        public static final Pose2d redAmp = new Pose2d(15.75, 6.55, new Rotation2d());
        public static final Pose2d redCenter = new Pose2d(15.27, 5.544, new Rotation2d());
        public static final Pose2d redSource = new Pose2d(15.75, 4.55, new Rotation2d());
    }
}