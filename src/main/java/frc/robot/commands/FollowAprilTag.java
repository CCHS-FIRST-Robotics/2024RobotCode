package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerveDrive.Drive;
import frc.robot.subsystems.vision.Vision;

public class FollowAprilTag extends CommandBase {

    Drive drive;
    Vision vision;

    private static double FOLLOWING_DISTANCE = 0.5; // meters
    Supplier<Transform2d> tagTransformSuppier;
    
    public FollowAprilTag(
        Drive drive,
        Vision vision
    ) {
        addRequirements(drive, vision);
        this.drive = drive;
        this.vision = vision;
        this.tagTransformSuppier = () -> vision.getTransformToClosestTag();
    }

    public FollowAprilTag(
        Drive drive,
        Vision vision,
        int tagId
    ) {
        addRequirements(drive, vision);
        this.drive = drive;
        this.vision = vision;
        this.tagTransformSuppier = () -> vision.getTagFromId(tagId).getTransform();
    }

    @Override
    public void execute() {
        // this took me wayyyy too long to write ive been working on it for like 45 minutes
        // tranform of the robot to the closest tag - ROBOT RELATIVE DISPLACEMENT
        Transform2d transformToTag = tagTransformSuppier.get();
        Pose2d currentPose = drive.getPose();

        Pose2d targetPose = currentPose
            .plus(transformToTag) // transform to tag (ie get field relative pose of tag)
            .plus(new Transform2d(new Translation2d(-FOLLOWING_DISTANCE, 0), new Rotation2d())); // move pose to FOLLOWING_DISTANCE in front of tag
        
        // create trajectory to send to drive -- just one point, might want to make a motion profile later
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();
        poseTrajectory.add(targetPose);
        velocityTrajectory.add(new Twist2d());

        drive.runPosition(poseTrajectory, velocityTrajectory);
    }
}
