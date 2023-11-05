package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerveDrive.Drive;
import frc.robot.subsystems.vision.Camera;

public class FollowAprilTag extends CommandBase {

    Drive drive;
    Camera camera;

    private static double FOLLOWING_DISTANCE = 0.5; // meters
    
    public FollowAprilTag(
        Drive drive,
        Camera camera 
    ) {
        addRequirements(drive, camera);
        this.drive = drive;
        this.camera = camera;
    }

    @Override
    public void execute() {
        // this took me wayyyy too long to write ive been working on it for like 45 minutes
        // tranform of the camera to the closest tag - ROBOT RELATIVE DISPLACEMENT
        Transform2d tagTransform = camera.getTransformToClosestTag();
        Pose2d currentPose = drive.getPose();

        Pose2d targetPose = currentPose
            .plus(tagTransform) // transform to tag (ie get field relative pose of tag)
            .plus(new Transform2d(new Translation2d(-FOLLOWING_DISTANCE, 0), new Rotation2d())); // move pose to FOLLOWING_DISTANCE in front of tag
        
        // create trajectory to send to drive -- just one point, might want to make a motion profile later
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();
        poseTrajectory.add(targetPose);
        velocityTrajectory.add(new Twist2d());
    }
}
