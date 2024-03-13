package frc.robot.subsystems.noteIO.intakeGround;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;

public class IntakeGround extends SubsystemBase {
    IntakeGroundIO io;
    Measure<Voltage> volts = Volts.of(0);
    private double startTime;
    IntakeGroundIOInputsAutoLogged inputs = new IntakeGroundIOInputsAutoLogged();

    public IntakeGround(IntakeGroundIO io) {
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
        Logger.processInputs("intake", inputs);
        Logger.recordOutput("Ground On", volts != Volts.of(0));

        if (checkNoteThere()) {
            stop();
        }

        io.setVoltage(volts);
    }

    private boolean checkNoteThere() {
        return inputs.motor1Current > 30 && Timer.getFPGATimestamp() - startTime > 0.1;
        // return inputs.motor1Current > 15 && inputs.motor1Velocity > 5000 * (volts /
        // 12);
    }

    public Command getIntakeCommand(Measure<Voltage> v) {
        // turns motor on until note detected
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteThere(),
                this);
    }

    public Command getHandNoteCommand(Measure<Voltage> v, BooleanSupplier noteHanded) {
        // turns motor on until note not detected
        // ! ^ This might not work in reality because the current might drop under even
        // ! though we still have the note in the intake
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                noteHanded,
                this);
    }
}