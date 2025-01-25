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
	Supplier<Double> linearSpeedMultiplierSupplier;
	Supplier<Rotation2d> headingAngleSupplier;

	double headingGoal;
	double prevHeadingSetpoint;
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

		angularProfile = new TrapezoidProfile(
						new Constraints(drive.getMaxAngularSpeed(), drive.getMaxAngularAcceleration()));
	}

	@Override
	public void initialize() {
		prevSpeeds = new ChassisSpeeds();
		headingController = drive.getHeadingController();
		drive.setOpenLoop(true);
	}

	@Override
	public void execute() {
		double linearXSpeed = linearXSpeedSupplier.get();
		double linearYSpeed = linearYSpeedSupplier.get();
		double linearSpeed = Math.hypot(linearXSpeed, linearYSpeed);
		Rotation2d linearDirection = new Rotation2d(linearXSpeed, linearYSpeed);

		double angularSpeed = angularSpeedSupplier.get();
		// System.out.println(angularSpeed);

		linearSpeed = applyPreferences(linearSpeed, HardwareConstants.LINEAR_SPEED_EXPONENT,
						Constants.ANALOG_DEADZONE);
		angularSpeed = applyPreferences(angularSpeed, HardwareConstants.ANGULAR_SPEED_EXPONENT,
						Constants.ANALOG_DEADZONE);

		linearSpeed *= linearSpeedMultiplierSupplier.get();

		// Calcaulate new linear components
		Translation2d linearVelocity = new Translation2d(linearSpeed, linearDirection);


        prevHeadingSetpoint = drive.getYaw().getRadians();
        prevHeadingSpeed = drive.getVelocity().dtheta;

        // calculate how far we would travel if we stopped now
        double getStoppingDistance = getStoppingDistance(prevHeadingSpeed,
                        drive.getMaxAngularAcceleration().in(RadiansPerSecond.per(Second)));
        headingGoal = prevHeadingSetpoint + getStoppingDistance * Math.signum(prevHeadingSpeed);
        headingGoal = MathUtil.angleModulus(headingGoal);
		
		Logger.recordOutput("Auto/JoystickHeadingGoal", headingGoal);
		Logger.recordOutput("Auto/JoystickHeadingSetpoint", prevHeadingSetpoint);
		Logger.recordOutput("Auto/JoystickHeadingVelSetpoint", prevHeadingSpeed);

		// Convert to meters per second
		ChassisSpeeds speeds = new ChassisSpeeds(
						linearVelocity.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
						linearVelocity.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
						angularSpeed * drive.getMaxAngularSpeed().in(RadiansPerSecond));

		// Convert from field relative
		var driveRotation = drive.getYaw();
		if (DriverStation.getAlliance().get() == Alliance.Red) {
				driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
		}
		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond,
						speeds.omegaRadiansPerSecond,
						driveRotation);

		// Desaturate speeds to max acceleration
		speeds = new ChassisSpeeds(
			MathUtil.clamp(
				speeds.vxMetersPerSecond,
				prevSpeeds.vxMetersPerSecond
					- drive.getMaxLinearAcceleration()
									.in(MetersPerSecondPerSecond)
									* Constants.PERIOD,
				prevSpeeds.vxMetersPerSecond
					+ drive.getMaxLinearAcceleration()
									.in(MetersPerSecondPerSecond)
									* Constants.PERIOD),
			MathUtil.clamp(
				speeds.vyMetersPerSecond,
				prevSpeeds.vyMetersPerSecond
					- drive.getMaxLinearAcceleration()
									.in(MetersPerSecondPerSecond)
									* Constants.PERIOD,
				prevSpeeds.vyMetersPerSecond
								+ drive.getMaxLinearAcceleration()
												.in(MetersPerSecondPerSecond)
												* Constants.PERIOD),
			MathUtil.clamp(
				speeds.omegaRadiansPerSecond,
				prevSpeeds.omegaRadiansPerSecond
					- drive.getMaxAngularAcceleration()
									.in(RadiansPerSecond.per(Second))
									* Constants.PERIOD,
				prevSpeeds.omegaRadiansPerSecond
					+ drive.getMaxAngularAcceleration()
									.in(RadiansPerSecond.per(Second))
									* Constants.PERIOD));
		prevSpeeds = speeds;

        Logger.recordOutput("targetVelocityField", speeds);
        
        drive.runVelocity(speeds);
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
}
