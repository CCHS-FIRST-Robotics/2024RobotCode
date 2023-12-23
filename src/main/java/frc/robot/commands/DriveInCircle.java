package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerveDrive.Drive;
import frc.robot.subsystems.swerveDrive.Drive.CONTROL_MODE;
import frc.robot.utils.DriveTrajectoryGenerator;
import frc.robot.utils.DriveTrajectory;


public class DriveInCircle extends Command {
    
    Drive drive;

    Supplier<Translation2d> centerSupplier;
    Supplier<Double> linearSpeedSupplier;
    Supplier<Double> angularSpeedSupplier;

    Pose2d previousPose;
    Translation2d centerTranslation;

    double prevLinearSpeed;
    double prevRotationalSpeed;

    public DriveInCircle(
        Drive drive, 
        Supplier<Translation2d> centerSupplier, 
        Supplier<Double> linearSpeedSupplier, 
        Supplier<Double> angularSpeedSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;

        this.centerSupplier = centerSupplier;
        this.linearSpeedSupplier = linearSpeedSupplier;
        this.angularSpeedSupplier = angularSpeedSupplier;
    }

    @Override
    public void initialize() {
        previousPose = drive.getPose(); // GETS UPDATED
        centerTranslation = previousPose.plus(new Transform2d(centerSupplier.get(), new Rotation2d())).getTranslation(); // CONSTANT

        prevLinearSpeed = 0;
        prevRotationalSpeed = 0;
    }

    @Override
    public void execute() {
        double radius = centerSupplier.get().getNorm(); // meters
        double linearSpeed = linearSpeedSupplier.get(); // meters per second
        double rotationalSpeed = angularSpeedSupplier.get(); // radians per second

        // Angular acceleration constraint
        rotationalSpeed = MathUtil.clamp(
            rotationalSpeed,
            prevRotationalSpeed - drive.getMaxAngularAccelerationRadPerSecPerSec() / 2 * Constants.PERIOD,
            prevRotationalSpeed + drive.getMaxAngularAccelerationRadPerSecPerSec() / 2 * Constants.PERIOD
        );
        prevRotationalSpeed = rotationalSpeed;

        // Adjust rotational speed to match (kidna) change in linear speed
        // ie., represent the rotational speed as proportional to the angular velocity of the orbit
        // and then scale it back after adjusting the linear speed
        double rotationalSpeedMultiplier = rotationalSpeed / (linearSpeed / radius);

        // Centripetal acceleration constraint
        if (linearSpeed*linearSpeed / radius > drive.getMaxLinearAccelerationMetersPerSecPerSec()) {
            // Adjust linear speed to limit centripetal acceleation
            linearSpeed = Math.sqrt(drive.getMaxLinearAccelerationMetersPerSecPerSec() * radius);
        }

        // Linear acceleration constraint (divided by 2, since I want to be extra careful not to lose traction)
        linearSpeed = MathUtil.clamp(
            linearSpeed,
            prevLinearSpeed - drive.getMaxLinearAccelerationMetersPerSecPerSec() / 2 * Constants.PERIOD,
            prevLinearSpeed + drive.getMaxLinearAccelerationMetersPerSecPerSec() / 2 * Constants.PERIOD
        );
        prevLinearSpeed = linearSpeed;

        // scale rot speed back after adjusting the linear speed
        rotationalSpeed = (linearSpeed / radius) * rotationalSpeedMultiplier;


        ArrayList<Pose2d> positionTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> velocityTrajectory = new ArrayList<Twist2d>();

        // for (int i=0; i<300; i++) { // FOR LOOP FOR TESTING ONLY
        // Move the robot's translation along the circle based on the desired linear speed
        Translation2d displacementFromCenter = previousPose.getTranslation().minus(centerTranslation);
        Translation2d targetTranslation = centerTranslation.plus(displacementFromCenter.rotateBy(new Rotation2d(linearSpeed / radius * Constants.PERIOD)));

        // Move the robot's rotation based on the desired rotational speed
        Rotation2d targetRotation = previousPose.getRotation().rotateBy(new Rotation2d(rotationalSpeed * Constants.PERIOD));

        Pose2d targetPose = new Pose2d(targetTranslation, targetRotation);

        Rotation2d displacementAngle = targetTranslation.minus(previousPose.getTranslation()).getAngle();
        Twist2d targetVelocity = new Twist2d(
            linearSpeed * displacementAngle.getCos(), 
            linearSpeed * displacementAngle.getSin(), 
            rotationalSpeed
        );

        // TODO: figure out why this doenst work??
        // Twist2d targetVelocity = previousPose.log(targetPose); // NOTE: generates the DISPLACEMENT twist, not a velocity
        // targetVelocity = new Twist2d(
        //     targetVelocity.dx / Constants.PERIOD, 
        //     targetVelocity.dy / Constants.PERIOD, 
        //     targetVelocity.dtheta / Constants.PERIOD
        // ); 

        previousPose = targetPose;
        
        positionTrajectory.add(targetPose);
        velocityTrajectory.add(targetVelocity);
        // }
        // DriveTrajectory trajectory = new DriveTrajectory(positionTrajectory, velocityTrajectory);
        // System.out.println("Writing trajectory to CSV");
        // trajectory.toCSV("circleTestTrajectory");

        drive.runPosition(positionTrajectory, velocityTrajectory);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("STOPPED");
        drive.setControlMode(CONTROL_MODE.CHASSIS_SETPOINT);
        drive.stop();
    }
}
