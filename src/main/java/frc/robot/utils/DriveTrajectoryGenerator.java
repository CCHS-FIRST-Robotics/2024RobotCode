package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import frc.robot.*;
import frc.robot.subsystems.mecaDrive.Drive;

public class DriveTrajectoryGenerator {
    
    public DriveTrajectoryGenerator() {

    }

    public static DriveTrajectory generateTrapezoidTrajectory(Pose2d targetPose, Twist2d targetVelocity, Pose2d currentPose, Twist2d currentVelocity, Constraints linearConstraints, Constraints angularConstraints) {

        // System.out.println("TESTING");
        // System.out.println(targetPose);
        // System.out.println(currentPose);

        TrapezoidProfile.State targetXState = new TrapezoidProfile.State(targetPose.getX(), targetVelocity.dx);
        TrapezoidProfile.State targetYState = new TrapezoidProfile.State(targetPose.getY(), targetVelocity.dy);
        TrapezoidProfile.State targetHeadingState = new TrapezoidProfile.State(targetPose.getRotation().getRadians(), targetVelocity.dtheta);

        TrapezoidProfile.State currentXState = new TrapezoidProfile.State(currentPose.getX(), currentVelocity.dx);
        TrapezoidProfile.State currentYState = new TrapezoidProfile.State(currentPose.getY(), currentVelocity.dy);
        TrapezoidProfile.State currentHeadingState = new TrapezoidProfile.State(currentPose.getRotation().getRadians(), currentVelocity.dtheta);

        var profileX = new TrapezoidProfile(linearConstraints, targetXState, currentXState);
        var profileY = new TrapezoidProfile(linearConstraints, targetYState, currentYState);
        var profileHeading = new TrapezoidProfile(angularConstraints, targetHeadingState, currentHeadingState);

        // Find the max time it takes to reach setpoint
        double timeToEnd = Math.max(Math.max(profileX.totalTime(), profileY.totalTime()), profileHeading.totalTime());

        // Create a list of poses and velocities (represented as twists) for each time step
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();
        // +2 so that the last point is included just in case (int) cuts it off
        for (int i = 1; i < (int) (timeToEnd / Constants.PERIOD) + 2; i++) {
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

        return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    }

    public static DriveTrajectory generateGuidedTrapezoidTrajectory(Pose2d targetPose, Twist2d targetVelocity, Pose2d currentPose, Twist2d currentVelocity, Constraints linearConstraints, Constraints angularConstraints, ArrayList<Pose2d> guidePoints) {
        var trajectories = new ArrayList<DriveTrajectory>();
        for (int i = 0; i < guidePoints.size() + 1; i++) {
            Pose2d startPoint = (i == 0) ? currentPose : guidePoints.get(i - 1);
            Pose2d endPoint = (i == guidePoints.size()) ? targetPose : guidePoints.get(i);

            Twist2d startVelocity = (i == 0) ? currentVelocity : new Twist2d();
            Twist2d endVelocity = (i == guidePoints.size()) ? targetVelocity : new Twist2d();

            trajectories.add(generateTrapezoidTrajectory(endPoint, endVelocity, startPoint, startVelocity, linearConstraints, angularConstraints));
        }
        
        DriveTrajectory finalTrajectory = new DriveTrajectory();
        for (DriveTrajectory trajectory : trajectories) {
            finalTrajectory.add(trajectory);
        }
        return finalTrajectory;
    }

    // public static DriveTrajectory generateSplineTrajectory(Pose2d targetPose, Twist2d targetVelocity, Pose2d currentPose, Twist2d currentVelocity, Constraints linearConstraints, Constraints angularConstraints, ArrayList<Pose2d> guidePoints) {
        
    // }

    // public static DriveTrajectory generateSinglePointTrajectory(Pose2d targetPose, Twist2d targetVelocity, Pose2d currentPose, Twist2d currentVelocity, Constraints constraints) {
        

    //     poseTrajectory.add(targetPose);
    //     velocityTrajectory.add(targetVelocity);

    //     return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    // }
}
