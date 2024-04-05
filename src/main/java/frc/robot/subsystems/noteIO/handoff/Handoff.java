package frc.robot.subsystems.noteIO.handoff;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Handoff extends SubsystemBase {
    private HandoffIO io;
    private Measure<Voltage> volts = Volts.of(0);
    private double startTime;
    private HandoffIOInputsAutoLogged inputs = new HandoffIOInputsAutoLogged();

    public Handoff(HandoffIO io) {
        this.io = io;
    }

    public void start(Measure<Voltage> v) {
        volts = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = Volts.of(0);
    }

    @AutoLogOutput
    public boolean checkNoteThere() {
        return inputs.motorCurrent > 25 && (Timer.getFPGATimestamp() - startTime > 0.5);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
        Logger.recordOutput("Handoff on", volts != Volts.of(0));

        io.setVoltage(volts);
    }

    // turns motor on until interrupted
    public Command getHandoffManualCommand(Measure<Voltage> v) {
        return new StartEndCommand(
                () -> start(v),
                this::stop,
                this);
    }

    // turns motor on until note detected
    public Command getHandoffCommand(Measure<Voltage> v) {
        return new StartEndCommand(
                () -> start(v),
                this::stop,
                this).until(this::checkNoteThere);
    }

    // turns motor on until shooter detects note
    public Command getShootCommand(Measure<Voltage> v, BooleanSupplier shooterDone) {
        return new StartEndCommand(
                () -> start(v),
                this::stop,
                this).until(shooterDone);
    }
}