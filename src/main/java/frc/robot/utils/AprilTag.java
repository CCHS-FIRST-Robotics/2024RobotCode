package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AprilTag {
    
    public int id;
    public Transform2d transform;

    /**
     * Constructs a new AprilTag object from the x, y, and heading of the robot to the tag
     * 
     * @param id The ID of the tag
     * @param x The x (forward) displacement from the robot to the tag
     * @param y The y (left) displacement from the robot to the tag
     * @param heading The heading of the robot to the tag
     */
    public AprilTag(int id, double x, double y, double heading) {
        this.id = id;
        this.transform = new Transform2d(
            new Translation2d(x, y),
            new Rotation2d(heading)
        ); 
    }

    /**
     * Constructs a new AprilTag object from a transform from the robot to the tag
     * 
     * @param id The ID of the tag
     * @param transform The transform from the robot to the tag
     */
    public AprilTag(int id, Transform2d transform) {
        this.id = id;
        this.transform = transform;
    }

    /**
     * Returns the ID of the tag
     * 
     * @return The ID of the tag
     */
    public double getId() {
        return id;
    }

    /**
     * Returns the transform from the robot to the tag
     * 
     * @return The transform from the robot to the tag
     */
    public Transform2d getTransform() {
        return transform;
    }

    /**
     * Returns the pose of the transform from robot to the tag
     * 
     * @return The pose of the robot to the tag
     */
    public Pose2d getPose2d() {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Returns the distance from the robot to the tag
     * 
     * @return The distance from the robot to the tag
     */
    public double getDistance() {
        return transform.getTranslation().getDistance(new Translation2d());
    }

    /**
     * Returns the x (forward) displacement from the robot to the tag
     * 
     * @return The x (forward) displacement from the robot to the tag
     */
    public double getX() {
        return transform.getTranslation().getX();
    }

    /**
     * Returns the y (left) displacement from the robot to the tag
     * 
     * @return The y (left) displacement from the robot to the tag
     */
    public double getY() {
        return transform.getTranslation().getY();
    }

    /**
     * Returns the heading of the robot to the tag
     * 
     * @return The heading of the robot to the tag
     */
    public double getHeading() {
        return transform.getRotation().getRadians();
    }
}
