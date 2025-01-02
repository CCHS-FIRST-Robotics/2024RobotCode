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
import frc.robot.Constants;
import frc.robot.HardwareConstants;

public class DriveWithJoysticks extends Command {
    Drive drive;
    Supplier<Double> linearXSpeedSupplier;
    Supplier<Double> linearYSpeedSupplier;
    Supplier<Double> angularSpeedSupplier;

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
        Supplier<Double> angularSpeedSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.linearXSpeedSupplier = linearXSpeedSupplier;
        this.linearYSpeedSupplier = linearYSpeedSupplier;
        this.angularSpeedSupplier = angularSpeedSupplier;
    }
    
    @Override
    public void initialize() {
        prevSpeeds = new ChassisSpeeds();

        angularProfile = new TrapezoidProfile(new Constraints(drive.getMaxAngularSpeed(), drive.getMaxAngularAcceleration()));
        headingController = drive.getHeadingController();
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


        // ! what does this do
        prevHeadingSetpoint = drive.getYaw().getRadians();
        prevHeadingSpeed = drive.getVelocity().dtheta;

        double stoppingDistance = getStoppingDistance(prevHeadingSpeed,
            drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second))
        );

        headingGoal = prevHeadingSetpoint + stoppingDistance * Math.signum(prevHeadingSpeed);
        headingGoal = MathUtil.angleModulus(headingGoal);
        // ! end of what does this do

        // make chassisspeeds object with FOC
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
            linearVelocity.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
            angularSpeed * drive.getMaxAngularSpeed().in(RadiansPerSecond),
            drive.getYawWithAllianceRotation()
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