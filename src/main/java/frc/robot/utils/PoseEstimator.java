package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;

public class PoseEstimator {
    
    double latestTimestamp = -1;
    Pose2d poseEstimate = new Pose2d();
    Pose3d poseEstimate3d = new Pose3d();

    public PoseEstimator() {

    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public Pose3d getPoseEstimate3d() {
        return poseEstimate3d;
    }

    public void addOdometryData(Twist2d odometryTwist, double timestamp) {
        latestTimestamp = timestamp;

        poseEstimate = poseEstimate.exp(odometryTwist);
        poseEstimate3d = poseEstimate3d.exp(new Twist3d(
            odometryTwist.dx, odometryTwist.dy, 0,
            0, 0, odometryTwist.dtheta
        ));
    }

    public void addVisionData(Pose3d visionPoseEstimate, double timestamp) {
        latestTimestamp = timestamp;

        poseEstimate = visionPoseEstimate.toPose2d();
        poseEstimate3d = visionPoseEstimate;
    }
    
}
