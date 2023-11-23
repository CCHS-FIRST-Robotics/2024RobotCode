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

    public AprilTag(int id, double x, double y, double heading) {
        this.id = id;
        this.transform = new Transform2d(
            new Translation2d(x, y),
            new Rotation2d(heading)
        ); 
    }

    public AprilTag(int id, Transform2d transform) {
        this.id = id;
        this.transform = transform;
    }

    public double getId() {
        return id;
    }

    public Transform2d getTransform() {
        return transform;
    }

    public Pose2d getPose2d() {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    public double getDistance() {
        return transform.getTranslation().getDistance(new Translation2d());
    }

    public double getX() {
        return transform.getTranslation().getX();
    }

    public double getY() {
        return transform.getTranslation().getY();
    }

    public double getHeading() {
        return transform.getRotation().getRadians();
    }
}
