package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.DriveTrajectoryGenerator;

// SHOULD (usually) BE RAN ONCE, CONTROL IS DONE IN DRIVE SUBSYSTEM
public class MoveToPose extends InstantCommand {

    Drive drive;
    Supplier<Pose2d> targetPoseSupplier;
    TrapezoidProfile profileX;
    TrapezoidProfile profileY;
    TrapezoidProfile profileHeading;
    TrapezoidProfile.Constraints linearConstraints;
    TrapezoidProfile.Constraints angularConstraints;

    public MoveToPose(
            Drive drive,
            Supplier<Pose2d> targetPoseSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        linearConstraints = new TrapezoidProfile.Constraints(drive.getMaxLinearSpeed().in(MetersPerSecond),
                drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond));
        angularConstraints = new TrapezoidProfile.Constraints(drive.getMaxAngularSpeed().in(RadiansPerSecond),
                drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second)));
    }

    @Override
    public void execute() {
        Pose2d targetPose = targetPoseSupplier.get();
        Twist2d targetVelocity = new Twist2d();
        Pose2d currentPose = drive.getPose();
        Twist2d currentVelocity = drive.getVelocity();

        System.out.println("TESTING");
        System.out.println(targetPose);
        System.out.println(currentPose);
        System.out.println(targetVelocity);
        System.out.println(currentVelocity);

        // TESTING GUIDE POINTS
        // var guidePoints = new ArrayList<Pose2d>();
        // guidePoints.add(new Pose2d(.25, .1, new Rotation2d(Math.PI)));
        // guidePoints.add(new Pose2d(-.75, -.5, new Rotation2d(Math.PI * 3 * .25)));
        // var driveTrajectory =
        // DriveTrajectoryGenerator.generateGuidedTrapezoidTrajectory(targetPose,
        // targetVelocity, currentPose, currentVelocity, linearConstraints,
        // angularConstraints, guidePoints);

        // TESTING CHOREO TRAJ
        // var driveTrajectory =
        // DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile("NewPath");

        var driveTrajectory = DriveTrajectoryGenerator.generateTrapezoidTrajectory(targetPose, targetVelocity,
                currentPose, currentVelocity, linearConstraints, angularConstraints);
        // System.out.println("Writing trajectory to CSV");
        // driveTrajectory.toCSV("choreoTrajectory");
        drive.runPosition(driveTrajectory.positionTrajectory, driveTrajectory.velocityTrajectory);
    }
}
