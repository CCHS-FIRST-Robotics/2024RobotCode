package frc.robot.subsystems.noteIO.intakeArm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
    IntakeArmIO io;
    double volts = 0;
    Debouncer currentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
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
        // returns whether current has risen for more than 0.1 seconds
        return currentDebouncer.calculate(inputs.motorCurrent > 15);

        // // returns whether note friction is detected and motor is up to speed
        // return inputs.motorCurrent > 15
        // && inputs.motorVelocity > (maxRPM / 60) * (volts /
        // 12);
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
        // turns motor on until shooter detects note
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                shooterDone,
                this);
    }
}