package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerveDrive.Drive;

// SHOULD (usually) BE RAN ONCE, CONTROL IS DONE IN DRIVE SUBSYSTEM
public class MoveToPose extends InstantCommand {

    Drive drive;
    Supplier<Pose2d> targetPoseSupplier;
    TrapezoidProfile profileX;
    TrapezoidProfile profileY;
    TrapezoidProfile profileHeading;
    TrapezoidProfile.Constraints constraints;

    public MoveToPose(
        Drive drive,
        Supplier<Pose2d> targetPoseSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        constraints = new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), drive.getMaxLinearAccelerationMetersPerSecPerSec());
    }

    @Override
    public void execute() {
        Pose2d targetPose = targetPoseSupplier.get();
        Pose2d currentPose = drive.getPose();
        Twist2d currentVelocity = drive.getVelocity();

        TrapezoidProfile.State targetXState = new TrapezoidProfile.State(targetPose.getX(), 0);
        TrapezoidProfile.State targetYState = new TrapezoidProfile.State(targetPose.getY(), 0);
        TrapezoidProfile.State targetHeadingState = new TrapezoidProfile.State(targetPose.getRotation().getRadians(), 0);

        TrapezoidProfile.State currentXState = new TrapezoidProfile.State(currentPose.getX(), currentVelocity.dx);
        TrapezoidProfile.State currentYState = new TrapezoidProfile.State(currentPose.getY(), currentVelocity.dy);
        TrapezoidProfile.State currentHeadingState = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), currentVelocity.dtheta);

        profileX = new TrapezoidProfile(constraints, targetXState, currentXState);
        profileY = new TrapezoidProfile(constraints, targetYState, currentYState);
        profileHeading = new TrapezoidProfile(constraints, targetHeadingState, currentHeadingState);

        // Find the max time it takes to reach setpoint
        double timeToEnd = Math.max(Math.max(profileX.totalTime(), profileY.totalTime()), profileHeading.totalTime());

        // Create a list of poses and velocities (represented as twists) for each time step
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();
        // +1 so that the last point is included just in case (int) cuts it off
        for (int i = 0; i < (int) (timeToEnd / Constants.PERIOD) + 2; i++) {
            double time = i * Constants.PERIOD;

            double x = profileX.calculate(time).position;
            double y = profileY.calculate(time).position;
            double heading = profileHeading.calculate(time).position;
            poseTrajectory.add(new Pose2d(x, y, new Rotation2d(heading)));  
            
            double dx = profileX.calculate(time).velocity;
            double dy = profileY.calculate(time).velocity;
            double dtheta = profileHeading.calculate(time).velocity;
            velocityTrajectory.add(new Twist2d(dx, dy, dtheta));

            // System.out.println(i);
            // System.out.println(new Pose2d(x, y, new Rotation2d(heading)));
            // System.out.println(new Twist2d(dx, dy, dtheta));
        }
        
        drive.runPosition(poseTrajectory, velocityTrajectory);
    }
}
