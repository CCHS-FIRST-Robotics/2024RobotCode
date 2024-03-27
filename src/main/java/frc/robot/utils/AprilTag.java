package frc.robot.utils;

import edu.wpi.first.math.geometry.*;

public class AprilTag {

    public int id;
    public Transform3d transform;

    /**
     * Constructs a new AprilTag object from the x, y, and heading of the robot to
     * the tag
     * 
     * @param id      The ID of the tag
     * @param x       The x (forward) displacement from the robot to the tag
     * @param y       The y (left) displacement from the robot to the tag
     * @param heading The heading of the robot to the tag
     */
    public AprilTag(int id, double x, double y, double heading) {
        this.id = id;
        this.transform = new Transform3d(
                new Translation3d(x, y, 0),
                new Rotation3d(0, 0, heading));
    }

    /**
     * Constructs a new AprilTag object from a transform from the robot to the tag
     * 
     * @param id        The ID of the tag
     * @param transform The transform from the robot to the tag
     */
    public AprilTag(int id, Transform2d transform) {
        this.id = id;
        this.transform = new Transform3d(
            new Translation3d(transform.getX(), transform.getY(), 0),
            new Rotation3d(0, 0, transform.getRotation().getRadians())
        );
    }

    /**
     * Constructs a new AprilTag object from a transform from the robot to the tag
     * 
     * @param id        The ID of the tag
     * @param transform The transform from the robot to the tag
     */
    public AprilTag(int id, Transform3d transform) {
        this.id = id;
        this.transform = transform;
    }

    /**
     * Constructs a new AprilTag object from a pose (transform) from the robot to
     * the tag
     * 
     * @param id        The ID of the tag
     * @param transform The pose (transform) from the robot to the tag
     */
    public AprilTag(int id, Pose3d pose) {
        this.id = id;
        this.transform = pose.minus(new Pose3d());
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
        return new Transform2d(transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
    }

    /**
     * Returns the pose of the transform from robot to the tag
     * 
     * @return The pose of the robot to the tag
     */
    public Pose2d getPose2d() {
        return getPose3d().toPose2d();
    }

    /**
     * Returns the pose of the transform from robot to the tag
     * 
     * @return The pose of the robot to the tag
     */
    public Pose3d getPose3d() {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Returns the distance from the robot to the tag
     * 
     * @return The distance from the robot to the tag
     */
    public double getDistance() {
        return transform.getTranslation().toTranslation2d().getDistance(new Translation2d());
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
        return transform.getRotation().getZ();
    }
}
