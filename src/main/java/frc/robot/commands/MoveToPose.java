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
        
        // System.out.println("TESTING");
        // System.out.println(targetPose);
        // System.out.println(currentPose);

        var driveTrajectory = DriveTrajectoryGenerator.generateTrapezoidTrajectory(targetPose, targetVelocity, currentPose, currentVelocity, constraints);
        drive.runPosition(driveTrajectory.positionTrajectory, driveTrajectory.velocityTrajectory);
    }
}
