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
import frc.robot.utils.DriveTrajectoryGenerator;
import frc.robot.utils.DriveTrajectory;

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
        Twist2d targetVelocity = new Twist2d();
        Pose2d currentPose = drive.getPose();
        Twist2d currentVelocity = drive.getVelocity();
        
        System.out.println("TESTING");
        System.out.println(targetPose);
        System.out.println(currentPose);
        System.out.println(targetVelocity);
        System.out.println(currentVelocity);

        // TESTING GUIDE POIITNS
        var guidePoints = new ArrayList<Pose2d>();
        guidePoints.add(new Pose2d(.25, .1, new Rotation2d(Math.PI)));
        guidePoints.add(new Pose2d(-.75, -.5, new Rotation2d(Math.PI * 3 * .75)));
        var driveTrajectory = DriveTrajectoryGenerator.generateGuidedTrapezoidTrajectory(targetPose, targetVelocity, currentPose, currentVelocity, constraints, guidePoints);

        // var driveTrajectory = DriveTrajectoryGenerator.generateTrapezoidTrajectory(targetPose, targetVelocity, currentPose, currentVelocity, constraints);
        System.out.println("Writing trajectory to CSV");
        driveTrajectory.toCSV();
        drive.runPosition(driveTrajectory.positionTrajectory, driveTrajectory.velocityTrajectory);
    }
}
