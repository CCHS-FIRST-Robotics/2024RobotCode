package frc.robot.subsystems.noteIO.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    Measure<Voltage> volts = Volts.of(0);
    private double startTime;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void start(Measure<Voltage> v) {
        volts = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = Volts.of(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake on", volts != Volts.of(0));

        io.setVoltage(volts);
    }

    public boolean checkNoteThere() {
        return inputs.motor1Current > 28 && Timer.getFPGATimestamp() - startTime > 0.1;
    }

    // turns motor on
    public Command getIntakeCommand(Measure<Voltage> v) {
        return startEnd(() -> start(v), this::stop);
    }

    // turns motor on until note detected
    public Command getIntakeCommand(Measure<Voltage> v, BooleanSupplier noteDetected) {
        return startEnd(() -> start(v), this::stop).until(noteDetected);
    }
}