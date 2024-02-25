package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.HardwareConstants;
import frc.robot.subsystems.drive.swerveDrive.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DriveWithWiimote extends Command {

    Drive drive;
    Supplier<Double> linearXSpeedSupplier;
    Supplier<Double> linearYSpeedSupplier;
    Supplier<Integer> linearSignSupplier;
    Supplier<Rotation2d> rotationSupplier;
    Supplier<Double> linearSpeedMultiplierSupplier;

    Rotation2d prevRotation;

    public DriveWithWiimote(
            Drive drive,
            Supplier<Double> rotateXSupplier,
            Supplier<Double> rotateYSupplier,
            Trigger onePressedSupplier,
            Trigger twoPressedSupplier,
            Supplier<Rotation2d> POVSupplier,
            Supplier<Double> linearSpeedMultiplierSupplier) {
        addRequirements(drive);
        this.drive = drive;
        linearYSpeedSupplier = rotateXSupplier;
        linearXSpeedSupplier = rotateYSupplier;
        linearSignSupplier = () -> getLinearSign(onePressedSupplier, twoPressedSupplier);
        rotationSupplier = POVSupplier;
        this.linearSpeedMultiplierSupplier = linearSpeedMultiplierSupplier;
    }

    @Override
    public void initialize() {
        prevRotation = new Rotation2d();
    }

    @Override
    public void execute() {
        double linearXSpeed = linearXSpeedSupplier.get();
        double linearYSpeed = linearYSpeedSupplier.get() * linearSignSupplier.get();

        Rotation2d rotation = rotationSupplier.get();
        prevRotation = rotation;

        double linearSpeed = (linearSignSupplier.get() == 0) ? 0 : Math.hypot(linearXSpeed, linearYSpeed);
        Rotation2d linearDirection = new Rotation2d(linearXSpeed, linearYSpeed);

        // TODO: switch constants to tunable numbers
        linearSpeed = applyPreferences(linearSpeed, HardwareConstants.LINEAR_SPEED_EXPONENT, Constants.ANALOG_DEADZONE);
        linearSpeed *= linearSpeedMultiplierSupplier.get();

        // Calcaulate new linear components
        Translation2d linearVelocity = new Translation2d(linearSpeed, linearDirection);

        // CONVERT INTO POSITION/VELOCITY FOR ROBOT TO FOLLOW
        // Pose2d currentPose = drive.getPose();
        // Pose2d targetPose = new Pose2d(currentPose.getTranslation(), rotation);

        // Twist2d currentVelocity = drive.getVelocity();

        // // constrain velocity to max speed
        // double rotError = rotation.getRadians() -
        // currentPose.getRotation().getRadians();
        // double rotVelocity = MathUtil.clamp(
        // rotError / Constants.PERIOD,
        // -drive.getMaxAngularSpeedRadPerSec(),
        // drive.getMaxAngularSpeedRadPerSec()
        // );
        // // constrain velocity to max acceleration
        // rotVelocity = MathUtil.clamp(
        // rotVelocity,
        // currentVelocity.dtheta - drive.getMaxAngularAccelerationRadPerSecPerSec() *
        // Constants.PERIOD,
        // currentVelocity.dtheta + drive.getMaxAngularAccelerationRadPerSecPerSec() *
        // Constants.PERIOD
        // );

        drive.runWii(linearVelocity, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("STOPPED");
        // drive.setControlMode(CONTROL_MODE.DISABLED);
        drive.stop();
    }

    private int getLinearSign(Trigger onePressedSupplier, Trigger twoPressedSupplier) {
        if (onePressedSupplier.getAsBoolean() && twoPressedSupplier.getAsBoolean()) {
            return 0;
        } else if (onePressedSupplier.getAsBoolean()) {
            return 1;
        } else if (twoPressedSupplier.getAsBoolean()) {
            return -1;
        } else {
            return 0;
        }
    }

    private double applyPreferences(double input, double exponent, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

}
