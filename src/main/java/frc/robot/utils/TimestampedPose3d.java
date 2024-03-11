package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;

public class TimestampedPose3d {
    public Pose3d pose;
    public double timestamp;

    public TimestampedPose3d(Pose3d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
