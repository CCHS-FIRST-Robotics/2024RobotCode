package frc.robot.subsystems.noteIO.intakeArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
    IntakeArmIO io;
    double volts = 0;
    Debouncer currentDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kRising);
    IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();
    double time;

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
    }

    public void start(double v) {
        volts = v;
        time = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = 0;
    }

    private boolean checkNoteThere() {
        // returns whether current has risen for more than 0.3 seconds
        // return currentDebouncer.calculate(inputs.motorCurrent > 20);
        return inputs.motorCurrent > 30 && (Timer.getFPGATimestamp() - time > .5);

        // // returns whether note friction is detected and motor is up to speed
        // return inputs.motorCurrent > 35
        // && inputs.motorVelocity > (98) * (volts /
        // 12d);
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