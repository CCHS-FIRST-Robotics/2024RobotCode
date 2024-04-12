package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

//! https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/networktables/package-summary.html 

public class TimestampedPose2d {
    public Pose2d pose;
    public double timestamp;

    public TimestampedPose2d(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}