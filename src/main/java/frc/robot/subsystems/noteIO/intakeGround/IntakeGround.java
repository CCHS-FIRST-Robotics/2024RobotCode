package frc.robot.subsystems.noteIO.intakeGround;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeGround extends SubsystemBase {
    IntakeGroundIO io;
    double volts = 0;
    IntakeGroundIOInputsAutoLogged inputs = new IntakeGroundIOInputsAutoLogged();

    public IntakeGround(IntakeGroundIO io) {
        this.io = io;
    }

    public void start(double v) {
        volts = v;
    }

    public void stop() {
        volts = 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);
        Logger.recordOutput("Ground On", volts != 0);

        if (checkNoteThere()) {
            volts = 0;
        }

        // io.setVoltage(volts);
    }

    private boolean checkNoteThere() {
        return inputs.motor1Current > 15 && inputs.motor1Velocity > 5000 * (volts / 12);
    }

    public Command startEndCommmand() {
        return startEnd(() -> start(12), () -> stop());
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

    public Command getHandNoteCommand(double v) {
        // turns motor on until note not detected
        // ! ^ This might not work in reality because the current might drop under even
        // ! though we still have the note in the intake
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> !checkNoteThere(),
                this);
    }
}