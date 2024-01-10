package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.swerveDrive.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

public class DriveWithJoysticks extends Command {

    Drive drive;
    Supplier<Double> linearXSpeedSupplier;
    Supplier<Double> linearYSpeedSupplier;
    Supplier<Double> angularSpeedSupplier;
    Supplier<Double> linearSpeedMultiplierSupplier;
    Supplier<Rotation2d> headingAngleSupplier;

    double headingGoal;
    @AutoLogOutput(key= "Auto/JoystickHeadingSetpoint")
    double prevHeadingSetpoint;
    @AutoLogOutput(key= "Auto/JoystickHeadingVelSetpoint")
    double prevHeadingSpeed;
    PIDController headingController;

    TrapezoidProfile angularProfile;

    ChassisSpeeds prevSpeeds;
    
    public DriveWithJoysticks(
        Drive drive, 
        Supplier<Double> leftXSupplier, 
        Supplier<Double> leftYSupplier, 
        Supplier<Double> rightXSupplier,
        Supplier<Double> linearSpeedMultiplierSupplier,
        Supplier<Rotation2d> headingSupplier 
    ) {
        addRequirements(drive);
        this.drive = drive;

        linearYSpeedSupplier = leftXSupplier;
        linearXSpeedSupplier = leftYSupplier;
        angularSpeedSupplier = rightXSupplier;

        this.linearSpeedMultiplierSupplier = linearSpeedMultiplierSupplier;
        
        headingAngleSupplier = headingSupplier;

        angularProfile = new TrapezoidProfile(new Constraints(drive.getMaxAngularSpeed(), drive.getMaxAngularAcceleration()));
    }

    @Override
    public void initialize() {
        prevSpeeds = new ChassisSpeeds();
        headingController = drive.getHeadingController();
    }

    @Override
    public void execute() {
        double linearXSpeed = linearXSpeedSupplier.get();
        double linearYSpeed = linearYSpeedSupplier.get();
        double linearSpeed = Math.hypot(linearXSpeed, linearYSpeed);
        Rotation2d linearDirection = new Rotation2d(linearXSpeed, linearYSpeed);
        
        double angularSpeed = angularSpeedSupplier.get();
        // System.out.println(angularSpeed);

        // TODO: switch constants to tunable numbers
        linearSpeed = applyPreferences(linearSpeed, Constants.LIENAR_SPEED_EXPONENT, Constants.ANALOG_DEADZONE);
        angularSpeed = applyPreferences(angularSpeed, Constants.ANGULAR_SPEED_EXPONENT, Constants.ANALOG_DEADZONE);
        // System.out.println(angularSpeed);

        linearSpeed *= linearSpeedMultiplierSupplier.get();

        // Calcaulate new linear components
        Translation2d linearVelocity = new Translation2d(linearSpeed, linearDirection);

        // APPLY ABSOLUTE HEADING CONTROL
        if (angularSpeed == 0) {
            headingGoal = headingAngleSupplier.get().getDegrees() == -1 ? headingGoal : headingAngleSupplier.get().getRadians();
            double currentHeadingRad = drive.getYaw().getRadians();
        
            State targetState = new State(headingGoal, 0);
            State currentState = new State(prevHeadingSetpoint, prevHeadingSpeed);
            optimizeStates(currentState, targetState, currentHeadingRad); // take shortest path to next angle

            State nextState = angularProfile.calculate(
                Constants.PERIOD, // calcaulate for next timestep
                currentState, // current state
                targetState // goal state
            );

            angularSpeed = nextState.velocity + headingController.calculate(drive.getYaw().getRadians(), nextState.position);
            // divide by max speed to get as a percentage of max (for continuity with joystick control)
            angularSpeed = angularSpeed / drive.getMaxAngularSpeed().in(RadiansPerSecond);

            prevHeadingSetpoint = nextState.position;
            prevHeadingSpeed = nextState.velocity;
        } else {
            headingGoal = drive.getYaw().getRadians();
        }

        // Convert to meters per second
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                linearVelocity.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                angularSpeed * drive.getMaxAngularSpeed().in(RadiansPerSecond)
            );

        // Convert from field relative
        var driveRotation = drive.getYaw();
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
        }

        // System.out.println("test");
        // System.out.println(speeds);

        // System.out.println(driveRotation);
        speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                driveRotation
            );

        // Desaturate speeds to max acceleration
        speeds =
        new ChassisSpeeds(
            MathUtil.clamp(
                speeds.vxMetersPerSecond,
                prevSpeeds.vxMetersPerSecond - drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD,
                prevSpeeds.vxMetersPerSecond + drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD),
            MathUtil.clamp(
                speeds.vyMetersPerSecond,
                prevSpeeds.vyMetersPerSecond - drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD,
                prevSpeeds.vyMetersPerSecond + drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD),
            MathUtil.clamp(
                speeds.omegaRadiansPerSecond,
                prevSpeeds.omegaRadiansPerSecond - drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second)) * Constants.PERIOD,
                prevSpeeds.omegaRadiansPerSecond + drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second)) * Constants.PERIOD)
        );
        prevSpeeds = speeds;

        // System.out.println(speeds.omegaRadiansPerSecond);
        
        drive.runVelocity(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double applyPreferences(double input, double exponent, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    private void optimizeStates(State currentState, State targetState, double currentHeading) {
        // Get error which is the smallest distance between goal and measurement
        double errorBound = (Math.PI - (-Math.PI)) / 2.0;
        double goalMinDistance =
            MathUtil.inputModulus(targetState.position - currentHeading, -errorBound, errorBound);
        double setpointMinDistance =
            MathUtil.inputModulus(currentState.position - currentHeading, -errorBound, errorBound);

        // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs to
        // be offset from the measurement by the input range modulus; they don't need to be equal.
        targetState.position = goalMinDistance + currentHeading;
        currentState.position = setpointMinDistance + currentHeading;
    }
}
