package frc.robot.subsystems.noteIO.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.HardwareConstants;

public class IntakeArm extends SubsystemBase {
    IntakeArmIO io;
    double volts = 0;
    IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
    }

    public void start(double v) {
        volts = v;
    }

    public void stop() {
        volts = 0;
    }

    private boolean checkNoteThere() {
        // returns whether note friction is detected and motor is up to speed
        return inputs.motorCurrent > 15
                && inputs.motorVelocity > (HardwareConstants.falcon500MaxRPM / 60) * (volts / 12);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        io.setVoltage(volts);
    }

    public Command getIntakeCommand(double v) {
        // turns motor on until note detected
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteThere(),
                this);
    }

    public Command getShootCommand(double v, BooleanSupplier shooterDone) {
        // turns motor on until shooter says note has been shot
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                shooterDone,
                this);
    }
}