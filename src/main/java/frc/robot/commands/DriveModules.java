package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.subsystems.drive.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.HardwareConstants;

public class DriveModules extends Command {
    Drive drive;
    Supplier<Double> linearSpeedSupplier;
    Supplier<Double> angularPositionSupplier;
    Supplier<Double> linearSpeedMultiplierSupplier;

    double prevSpeed;

    public DriveModules(
            Drive drive,
            Supplier<Double> leftYSupplier,
            Supplier<Double> rightXSupplier,
            Supplier<Double> linearSpeedMultiplierSupplier) {
        addRequirements(drive);
        this.drive = drive;
        linearSpeedSupplier = leftYSupplier;
        angularPositionSupplier = rightXSupplier;
        this.linearSpeedMultiplierSupplier = linearSpeedMultiplierSupplier;
    }

    @Override
    public void initialize() {
        prevSpeed = 0;
    }

    @Override
    public void execute() {
        double linearSpeed = linearSpeedSupplier.get();
        double angularPosition = angularPositionSupplier.get();

        angularPosition = applyPreferences(angularPosition, 1, Constants.ANALOG_DEADZONE);

        // TODO: switch constants to tunable numbers
        linearSpeed = applyPreferences(linearSpeed, HardwareConstants.LINEAR_SPEED_EXPONENT, Constants.ANALOG_DEADZONE);
        linearSpeed *= linearSpeedMultiplierSupplier.get();

        linearSpeed *= drive.getMaxLinearSpeed().in(MetersPerSecond);

        linearSpeed = MathUtil.clamp(
                linearSpeed,
                prevSpeed - drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD,
                prevSpeed + drive.getMaxLinearAcceleration().in(MetersPerSecondPerSecond) * Constants.PERIOD);

        drive.runModules(new SwerveModuleState(
                0, // linearSpeed
                new Rotation2d(Math.PI / 2.0 * angularPosition)));
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
}
