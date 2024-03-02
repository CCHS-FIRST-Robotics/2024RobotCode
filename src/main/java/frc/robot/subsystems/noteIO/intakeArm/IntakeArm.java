package frc.robot.subsystems.noteIO.intakeArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
    IntakeArmIO io;
    double volts = 0;
    double startTime;
    // Debouncer currentDebouncer = new Debouncer(0.3,
    // Debouncer.DebounceType.kRising);
    IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
    }

    public void start(double v) {
        volts = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = 0;
    }

    private boolean checkNoteThere() {
        return inputs.motorCurrent > 30 && (Timer.getFPGATimestamp() - startTime > 0.5);
        // return currentDebouncer.calculate(inputs.motorCurrent > 30);
        // return inputs.motorCurrent > 30 && inputs.motorVelocity > 98 * (volts / 12);
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

    public Command getShootCommand(double volts, BooleanSupplier shooterDone) {
        // turns motor on until shooter detects note
        return new FunctionalCommand(
                () -> start(volts),
                () -> {
                },
                (interrupted) -> stop(),
                shooterDone,
                this);
    }
}