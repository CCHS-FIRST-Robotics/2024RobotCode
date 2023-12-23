package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerveDrive.Drive;
import frc.robot.subsystems.swerveDrive.Drive.CONTROL_MODE;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.DriveTrajectoryGenerator;

public class FollowAprilTag extends Command {

    Drive drive;
    Vision vision;

    private static double FOLLOWING_DISTANCE = 1; // meters
    Supplier<Transform2d> tagTransformSuppier;
    Constraints linearConstraints;
    Constraints angularConstraints;

    Twist2d prevVelocity = new Twist2d();
    
    public FollowAprilTag(
        Drive drive,
        Vision vision
    ) {
        addRequirements(drive, vision);
        this.drive = drive;
        this.vision = vision;
        this.tagTransformSuppier = () -> vision.getTransformToClosestTag();
        linearConstraints = new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearAccelerationMetersPerSecPerSec());
        angularConstraints = new TrapezoidProfile.Constraints(drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularAccelerationRadPerSecPerSec());
    }

    public FollowAprilTag(
        Drive drive,
        Vision vision,
        int tagId
    ) {
        addRequirements(drive, vision);
        this.drive = drive;
        this.vision = vision;
        // this.tagTransformSuppier = () -> vision.getTagFromId(tagId).getTransform();
        linearConstraints = new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearAccelerationMetersPerSecPerSec());
        angularConstraints = new TrapezoidProfile.Constraints(drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularAccelerationRadPerSecPerSec());
    }

    @Override
    public void execute() {
        // System.out.println("RUNNING");
        // this took me wayyyy too long to write ive been working on it for like 45 minutes

        // tranform of the robot to the closest tag - ROBOT RELATIVE DISPLACEMENT
        Transform2d transformToTag = tagTransformSuppier.get();
        Pose2d currentPose = drive.getPose();
        Twist2d currentVelocity = drive.getVelocity();
        if (transformToTag.getX() < 0) {
            end(false);
            return;
        }

        Pose2d targetPose = currentPose
            .plus(transformToTag) // transform to tag (ie get field relative pose of tag)
            .plus(new Transform2d(new Translation2d(-FOLLOWING_DISTANCE, 0), new Rotation2d())); // move pose to FOLLOWING_DISTANCE in front of tag
        
        Twist2d targetVelocity = new Twist2d(
            (targetPose.getX() - currentPose.getX()) / Constants.PERIOD,
            (targetPose.getY() - currentPose.getY()) / Constants.PERIOD,
            0
            // (targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()) / Constants.PERIOD
        );
        targetPose = new Pose2d(targetPose.getTranslation(), currentPose.getRotation());

        // Constrain to max accel
        targetVelocity = new Twist2d(
            MathUtil.clamp(
                targetVelocity.dx,
                prevVelocity.dx - drive.getMaxLinearAccelerationMetersPerSecPerSec() * Constants.PERIOD,
                prevVelocity.dx + drive.getMaxLinearAccelerationMetersPerSecPerSec() * Constants.PERIOD
            ),
            MathUtil.clamp(
                targetVelocity.dy,
                prevVelocity.dy - drive.getMaxLinearAccelerationMetersPerSecPerSec() * Constants.PERIOD,
                prevVelocity.dy + drive.getMaxLinearAccelerationMetersPerSecPerSec() * Constants.PERIOD
            ),
            MathUtil.clamp(
                targetVelocity.dtheta,
                prevVelocity.dtheta - drive.getMaxAngularAccelerationRadPerSecPerSec() * Constants.PERIOD,
                prevVelocity.dtheta + drive.getMaxAngularAccelerationRadPerSecPerSec() * Constants.PERIOD
            )
        );
        prevVelocity = targetVelocity;
        
        // Twist2d targetVelocity = new Twist2d();
        // Pose2d targetPose = new Pose2d(FOLLOWING_DISTANCE, 0, transformToTag.getRotation().times(-1));
        

        // create trajectory to send to drive -- just one point, might want to make a motion profile later
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();
        poseTrajectory.add(targetPose);
        velocityTrajectory.add(targetVelocity);
        
        // System.out.println("VISION GOTO POSE:");
        // System.out.println(currentPose);
        // System.out.println(targetPose);

        drive.runPosition(poseTrajectory, velocityTrajectory);

        // var driveTrajectory = DriveTrajectoryGenerator.generateTrapezoidTrajectory(targetPose, targetVelocity, currentPose, currentVelocity, constraints);
        // drive.runPosition(driveTrajectory.positionTrajectory, driveTrajectory.velocityTrajectory);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("STOPPED");
        drive.setControlMode(CONTROL_MODE.CHASSIS_SETPOINT);
        drive.stop();
    }
}
