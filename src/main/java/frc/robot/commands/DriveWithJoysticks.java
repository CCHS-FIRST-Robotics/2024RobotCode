package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.HardwareConstants;

public class DriveWithJoysticks extends Command {
    Drive drive;
    Supplier<Double> linearXSpeedSupplier;
    Supplier<Double> linearYSpeedSupplier;
    Supplier<Double> angularSpeedSupplier;

    boolean usingHeadingFeedback;
    Supplier<Rotation2d> headingSupplier;

    double headingGoal;
    double prevHeadingSetpoint;
    double prevHeadingSpeed;
    
    TrapezoidProfile angularProfile;
    PIDController headingController;
    ChassisSpeeds prevSpeeds;

    public DriveWithJoysticks(
        Drive drive,
        Supplier<Double> linearXSpeedSupplier,
        Supplier<Double> linearYSpeedSupplier,
        Supplier<Double> angularSpeedSupplier,
        boolean usingHeadingFeedback,
        Supplier<Rotation2d> headingSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;

        this.linearXSpeedSupplier = linearXSpeedSupplier;
        this.linearYSpeedSupplier = linearYSpeedSupplier;
        this.angularSpeedSupplier = angularSpeedSupplier;

        this.usingHeadingFeedback = usingHeadingFeedback;
        this.headingSupplier = headingSupplier;
    }
    
    @Override
    public void initialize() {
        angularProfile = new TrapezoidProfile(new Constraints(drive.getMaxAngularSpeed(), drive.getMaxAngularAcceleration()));
        headingController = drive.getHeadingController();
        
        prevSpeeds = new ChassisSpeeds();
    }

    @Override
    public void execute() {
        double linearXSpeed = linearXSpeedSupplier.get();
        double linearYSpeed = linearYSpeedSupplier.get();
        double linearSpeed = Math.hypot(linearXSpeed, linearYSpeed);
        Rotation2d linearDirection = new Rotation2d(linearXSpeed, linearYSpeed);
        linearSpeed = applyPreferences(linearSpeed, HardwareConstants.LINEAR_SPEED_EXPONENT, Constants.ANALOG_DEADZONE);
        Translation2d linearVelocity = new Translation2d(
            linearSpeed, 
            linearDirection
        );

        double angularSpeed = angularSpeedSupplier.get();
        angularSpeed = applyPreferences(angularSpeed, HardwareConstants.ANGULAR_SPEED_EXPONENT, Constants.ANALOG_DEADZONE);

        if (usingHeadingFeedback 
            && angularSpeed == 0 // if the driver isn't turning
        ) {
            // ! what does this do
            headingGoal = headingSupplier.get().getDegrees() == -1 ? headingGoal : MathUtil.angleModulus(headingSupplier.get().getRadians());

            State currentState = new State(prevHeadingSetpoint, prevHeadingSpeed);
            State finalTargetState = new State(headingGoal, 0);
            State nextState = angularProfile.calculate(
                Constants.PERIOD,
                currentState,
                finalTargetState
            );

            angularSpeed = headingController.calculate(drive.getYaw().getRadians(), headingGoal);

            if (headingController.atSetpoint()){
                angularSpeed = 0;
            }
            // divide by max speed to get as a percentage of max (for continuity with
            // joystick control)
            angularSpeed = angularSpeed / drive.getMaxAngularSpeed().in(RadiansPerSecond);

            prevHeadingSetpoint = nextState.position;
            prevHeadingSpeed = nextState.velocity;
        } else {
            prevHeadingSetpoint = drive.getYaw().getRadians();
            prevHeadingSpeed = drive.getVelocity().dtheta;

            double stoppingDistance = getStoppingDistance(prevHeadingSpeed,
                drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second))
            );

            headingGoal = prevHeadingSetpoint + stoppingDistance * Math.signum(prevHeadingSpeed);
            headingGoal = MathUtil.angleModulus(headingGoal);
        }

        // ! should probably be in drive.getyaw
        // make field relative to red if on red team
        Rotation2d driveRotation = drive.getYaw().plus(
            DriverStation.getAlliance().get() == Alliance.Red ? 
            new Rotation2d(Math.PI) : 
            new Rotation2d(0)
        );

        // make chassisspeeds object with FOC
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
            linearVelocity.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
            angularSpeed * drive.getMaxAngularSpeed().in(RadiansPerSecond),
            driveRotation
        );

        // ! check colin's units
        speeds = new ChassisSpeeds(
            clampWithMaxAcceleration(
                speeds.vxMetersPerSecond, 
                prevSpeeds.vxMetersPerSecond, 
                drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD
            ),
            clampWithMaxAcceleration(
                speeds.vyMetersPerSecond, 
                prevSpeeds.vyMetersPerSecond, 
                drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD
            ),
            clampWithMaxAcceleration(
                speeds.omegaRadiansPerSecond, 
                prevSpeeds.omegaRadiansPerSecond, 
                drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second)) * Constants.PERIOD
            )
        );
        
        drive.runVelocity(speeds);
        prevSpeeds = speeds;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        drive.setOpenLoop(false);
    }

    private double applyPreferences(double input, double exponent, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    public double getStoppingDistance(double currentSpeed, double maxAcceleration) {
        return 0.5 * Math.pow(currentSpeed, 2) / maxAcceleration;
    }

    public double clampWithMaxAcceleration(double velocity, double prevVelocity, double maxAcceleration){
        return MathUtil.clamp(velocity, prevVelocity - maxAcceleration, prevVelocity + maxAcceleration);
    }
}