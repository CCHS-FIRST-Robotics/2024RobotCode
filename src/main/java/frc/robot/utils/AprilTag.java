package frc.robot.utils;

import edu.wpi.first.math.geometry.*;

public class AprilTag { 
    private int id;
    private Transform3d transform;

    public AprilTag(int id, Pose3d pose) {
        this.id = id;
        this.transform = pose.minus(new Pose3d());
    }

    public AprilTag(int id, Transform3d transform) {
        this.id = id;
        this.transform = transform;
    }
    
    public int getId(){
        return this.id;
    }

    public Transform3d getTransform3d(){
        return this.transform;
    }

    public Pose3d getPose3d() {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public Pose2d getPose2d() {
        return getPose3d().toPose2d();
    }

    public double getDistance() {
        return transform.getTranslation().toTranslation2d().getDistance(new Translation2d());
    }
}