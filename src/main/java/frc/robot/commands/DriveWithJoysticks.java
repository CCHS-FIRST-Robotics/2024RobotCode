package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.*;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.HardwareConstants;

public class DriveWithJoysticks extends Command {
    Drive drive;
    Supplier<Double> linearXSpeedSupplier;
    Supplier<Double> linearYSpeedSupplier;
    Supplier<Double> angularVelocitySupplier;

    double headingGoal;
    double prevHeadingSetpoint;
    double prevHeadingSpeed;
    
    ChassisSpeeds prevSpeeds;

    public DriveWithJoysticks(
        Drive drive,
        Supplier<Double> linearXSpeedSupplier,
        Supplier<Double> linearYSpeedSupplier,
        Supplier<Double> angularVelocitySupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.linearXSpeedSupplier = linearXSpeedSupplier;
        this.linearYSpeedSupplier = linearYSpeedSupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
    }

    @Override
    public void execute() {
        // get linearVelocity
        double linearXSpeed = linearXSpeedSupplier.get();
        double linearYSpeed = linearYSpeedSupplier.get();
        double linearSpeed = applyPreferences(Math.hypot(linearXSpeed, linearYSpeed), Constants.JOYSTICK_DEADZONE, Constants.LINEAR_SPEED_EXPONENT);
        Rotation2d linearDirection = new Rotation2d(linearXSpeed, linearYSpeed); // kinda weird lol
        Translation2d linearVelocity = new Translation2d(
            linearSpeed, 
            linearDirection
        );

        // get angularSpeed
        double angularVelocity = applyPreferences(-angularVelocitySupplier.get(), Constants.JOYSTICK_DEADZONE, Constants.ANGULAR_SPEED_EXPONENT);

        // FOC
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * HardwareConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
            linearVelocity.getY() * HardwareConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
            angularVelocity * HardwareConstants.MAX_ANGULAR_SPEED.in(RadiansPerSecond),
            drive.getYawWithAllianceRotation()
        );

        // clamp everything between max and min possible velocities
        speeds = new ChassisSpeeds(
            clampVelocity(
                speeds.vxMetersPerSecond, 
                prevSpeeds.vxMetersPerSecond, 
                HardwareConstants.MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) * Constants.PERIOD
            ),
            clampVelocity(
                speeds.vyMetersPerSecond, 
                prevSpeeds.vyMetersPerSecond, 
                HardwareConstants.MAX_LINEAR_ACCEL.in(MetersPerSecondPerSecond) * Constants.PERIOD
            ),
            clampVelocity(
                speeds.omegaRadiansPerSecond, 
                prevSpeeds.omegaRadiansPerSecond, 
                HardwareConstants.MAX_ANGULAR_ACCEL.in(RadiansPerSecond.per(Second)) * Constants.PERIOD
            )
        );
        
        drive.runVelocity(speeds);
        prevSpeeds = speeds;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double applyPreferences(double input, double deadzone, double exponent) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    public double getStoppingDistance(double currentSpeed, double maxAcceleration) {
        return 0.5 * Math.pow(currentSpeed, 2) / maxAcceleration;
    }

    public double clampVelocity(double velocity, double prevVelocity, double maxAcceleration){
        return MathUtil.clamp(velocity, prevVelocity - maxAcceleration, prevVelocity + maxAcceleration);
    }
}