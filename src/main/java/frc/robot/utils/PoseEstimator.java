package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;

public class PoseEstimator {
    
    double latestTimestamp = -1;
    Pose2d poseEstimate = new Pose2d();
    Pose3d poseEstimate3d = new Pose3d();

    /**
     * Constructs a new PoseEstimator object
     */
    public PoseEstimator() {

    }

    /**
     * Returns the latest 2d pose estimate
     * 
     * @return The latest 2d pose estimate
     */
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    /**
     * Returns the latest 3d pose estimate
     * 
     * @return The latest 3d pose estimate
     */
    public Pose3d getPoseEstimate3d() {
        return poseEstimate3d;
    }

    /**
     * Adds odometry data to the pose estimator (pose exponential)
     * 
     * @param odometryTwist The twist of the robot since the last odometry update
     * @param timestamp The timestamp of the odometry update
     */
    public void addOdometryData(Twist2d odometryTwist, double timestamp) {
        latestTimestamp = timestamp;

        poseEstimate = poseEstimate.exp(odometryTwist);
        poseEstimate3d = poseEstimate3d.exp(new Twist3d(
            odometryTwist.dx, odometryTwist.dy, 0,
            0, 0, odometryTwist.dtheta
        ));
    }

    /**
     * Adds vision data to the pose estimator (overriding the pose estimate)
     * 
     * @param visionPoseEstimate The pose estimate from vision
     * @param timestamp The timestamp of the vision update
     */
    public void addVisionData(Pose3d visionPoseEstimate, double timestamp) {
        latestTimestamp = timestamp;

        // This assumes that the vision pose estimate is in the same frame as the gyro
        // Also, kinda assumes that the pose estimate rotation is the same as the gyro rotation
        // (which it should be when there are just these two sources of data and the other is overriding this one)
        Rotation2d gyroAngle = poseEstimate.getRotation();

        poseEstimate = new Pose2d(visionPoseEstimate.toPose2d().getTranslation(), gyroAngle);
        poseEstimate3d = new Pose3d(
            visionPoseEstimate.getTranslation(), 
            new Rotation3d(
                visionPoseEstimate.getRotation().getX(), 
                visionPoseEstimate.getRotation().getY(),
                gyroAngle.getRadians()
            )
        );
    }
    
}
