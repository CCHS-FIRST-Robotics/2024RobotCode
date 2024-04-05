package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.choreo.lib.*;
import frc.robot.*;
import frc.robot.Constants.AutoPathConstants;

import java.util.ArrayList;

public final class DriveTrajectoryGenerator {

    private DriveTrajectoryGenerator() {

    }

    public static DriveTrajectory generateTrapezoidTrajectory2(
            Pose2d targetPose, Twist2d targetVelocity, Pose2d currentPose, Twist2d currentVelocity,
            Constraints linearConstraints, Constraints angularConstraints) {

        // System.out.println("TESTING");
        // System.out.println(targetPose);
        // System.out.println(currentPose);

        TrapezoidProfile.State targetXState = new TrapezoidProfile.State(targetPose.getX(), targetVelocity.dx);
        TrapezoidProfile.State targetYState = new TrapezoidProfile.State(targetPose.getY(), targetVelocity.dy);
        TrapezoidProfile.State targetHeadingState = new TrapezoidProfile.State(targetPose.getRotation().getRadians(),
                targetVelocity.dtheta);

        TrapezoidProfile.State currentXState = new TrapezoidProfile.State(currentPose.getX(), currentVelocity.dx);
        TrapezoidProfile.State currentYState = new TrapezoidProfile.State(currentPose.getY(), currentVelocity.dy);
        TrapezoidProfile.State currentHeadingState = new TrapezoidProfile.State(currentPose.getRotation().getRadians(),
                currentVelocity.dtheta);

        var profileX = new TrapezoidProfile(linearConstraints);
        var profileY = new TrapezoidProfile(linearConstraints);
        var profileHeading = new TrapezoidProfile(angularConstraints);

        // Find the max time it takes to reach setpoint
        double timeToEnd = Math.max(Math.max(profileX.totalTime(), profileY.totalTime()), profileHeading.totalTime());

        // Create a list of poses and velocities (represented as twists) for each time
        // step
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();
        // +2 so that the last point is included just in case (int) cuts it off
        for (int i = 1; i < (int) (timeToEnd / Constants.PERIOD) + 2; i++) {
            double time = i * Constants.PERIOD;

            double x = profileX.calculate(time, currentXState, targetXState).position;
            double y = profileY.calculate(time, currentYState, targetYState).position;
            double heading = profileHeading.calculate(time, currentHeadingState, targetHeadingState).position;
            poseTrajectory.add(new Pose2d(x, y, new Rotation2d(heading)));

            double dx = profileX.calculate(time, currentXState, targetXState).velocity;
            double dy = profileY.calculate(time, currentYState, targetYState).velocity;
            double dtheta = profileHeading.calculate(time, currentHeadingState, targetHeadingState).velocity;
            velocityTrajectory.add(new Twist2d(dx, dy, dtheta));

            // System.out.println(i);
            // System.out.println(new Pose2d(x, y, new Rotation2d(heading)));
            // System.out.println(new Twist2d(dx, dy, dtheta));
        }

        return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    }

    public static DriveTrajectory generateTrapezoidTrajectory(Pose2d targetPose, Twist2d targetVelocity,
            Pose2d currentPose, Twist2d currentVelocity, Constraints linearConstraints,
            Constraints angularConstraints) {

        // System.out.println("TESTING");
        // System.out.println(targetPose);
        // System.out.println(currentPose);

        TrapezoidProfile.State targetHeadingState = new TrapezoidProfile.State(targetPose.getRotation().getRadians(),
                targetVelocity.dtheta);
        TrapezoidProfile.State currentHeadingState = new TrapezoidProfile.State(currentPose.getRotation().getRadians(),
                currentVelocity.dtheta);

        var profileHeading = new TrapezoidProfile(angularConstraints);

        Translation2d[] translationTrajectory = new QuadraticProfile(Constants.PERIOD)
                .getCombinedSetPoints(
                        currentPose.getTranslation(),
                        targetPose.getTranslation(),
                        linearConstraints.maxVelocity,
                        linearConstraints.maxAcceleration);

        // Find the max time it takes to reach setpoint
        double timeToEnd = Math.max(translationTrajectory.length * Constants.PERIOD, profileHeading.totalTime());

        // Create a list of poses and velocities (represented as twists) for each time
        // step
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();

        var prevTranslation = currentPose.getTranslation();
        // +2 so that the last point is included just in case (int) cuts it off
        for (int i = 1; i < (int) (timeToEnd / Constants.PERIOD) + 2; i++) {
            double time = i * Constants.PERIOD;

            // Tranlation2d[] wont allow samples past the end of the trajectory, so use
            // different index
            int j = i;
            if (j > translationTrajectory.length - 1) {
                j = translationTrajectory.length - 1;
            }

            Translation2d translation = translationTrajectory[j];
            double heading = profileHeading.calculate(time, currentHeadingState, targetHeadingState).position;

            poseTrajectory.add(new Pose2d(translation.getX(), translation.getY(), new Rotation2d(heading)));

            double dtheta = profileHeading.calculate(time, currentHeadingState, targetHeadingState).velocity;
            velocityTrajectory.add(new Twist2d(
                    (translation.getX() - prevTranslation.getX()) / Constants.PERIOD,
                    (translation.getY() - prevTranslation.getY()) / Constants.PERIOD,
                    dtheta));

            prevTranslation = translation;

            // System.out.println(i);
            // System.out.println(new Pose2d(x, y, new Rotation2d(heading)));
            // System.out.println(new Twist2d(dx, dy, dtheta));
        }

        return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    }

    public static DriveTrajectory generateGuidedTrapezoidTrajectory(
            Pose2d targetPose, Twist2d targetVelocity, Pose2d currentPose, Twist2d currentVelocity,
            Constraints linearConstraints, Constraints angularConstraints, ArrayList<Pose2d> guidePoints) {
        var trajectories = new ArrayList<DriveTrajectory>();
        for (int i = 0; i < guidePoints.size() + 1; i++) {
            Pose2d startPoint = (i == 0) ? currentPose : guidePoints.get(i - 1);
            Pose2d endPoint = (i == guidePoints.size()) ? targetPose : guidePoints.get(i);

            Twist2d startVelocity = (i == 0) ? currentVelocity : new Twist2d();
            Twist2d endVelocity = (i == guidePoints.size()) ? targetVelocity : new Twist2d();

            trajectories.add(generateTrapezoidTrajectory(
                    endPoint, endVelocity, startPoint, startVelocity, linearConstraints, angularConstraints));
        }

        DriveTrajectory finalTrajectory = new DriveTrajectory();
        for (DriveTrajectory trajectory : trajectories) {
            finalTrajectory.add(trajectory);
        }
        return finalTrajectory;
    }

    public static DriveTrajectory generateChoreoTrajectoryFromFile(String path) {
        // ChoreoTrajectory choreoTrajectory = AutoPathConstants.ALLIANCE == Alliance.Red ? Choreo.getTrajectory(path).flipped() : Choreo.getTrajectory(path);
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);

        double timeToEnd = choreoTrajectory.getTotalTime();

        // Create a list of poses and velocities (represented as twists) for each time
        // step
        ArrayList<Pose2d> poseTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();

        for (int i = 0; i < (int) (timeToEnd / Constants.PERIOD) + 2; i++) {
            double time = i * Constants.PERIOD;

            ChoreoTrajectoryState state = choreoTrajectory.sample(time);
            poseTrajectory.add(new Pose2d(state.x, state.y, new Rotation2d(state.heading)));
            velocityTrajectory.add(new Twist2d(state.velocityX, state.velocityY, state.angularVelocity));
        }

        return new DriveTrajectory(poseTrajectory, velocityTrajectory);
    }
}
