package frc.robot.subsystems.noteIO.intakeArm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
    private IntakeArmIO io;
    private Measure<Voltage> volts = Volts.of(0);
    private double startTime;
    // Debouncer currentDebouncer = new Debouncer(0.3,
    // Debouncer.DebounceType.kRising);
    private IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
    }

    public void start(Measure<Voltage> v) {
        volts = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = Volts.of(0);
    }

    private boolean checkNoteThere() {
        return inputs.motorCurrent > 30 && (Timer.getFPGATimestamp() - startTime > 0.5);
        // return currentDebouncer.calculate(inputs.motorCurrent > 30);
        // return inputs.motorCurrent > 30 && inputs.motorVelocity > 98 * (volts / 12);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeArm", inputs);

        io.setVoltage(volts);
    }

    // turns motor on until note detected
    public Command getArmIntakeCommand(Measure<Voltage> v) {
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteThere(),
                this);
    }

    // turns motor on until shooter detects note
    public Command getShootCommand(Measure<Voltage> v, BooleanSupplier shooterDone) {
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                shooterDone,
                this);
    }
}