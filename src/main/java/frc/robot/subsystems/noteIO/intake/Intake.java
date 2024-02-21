package frc.robot.subsystems.noteIO.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    int volts = 0;
    boolean buttonHeld;
    boolean noteThere;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void start(int v) {
        volts = v;
        buttonHeld = true;
    }

    public void stop() {
        buttonHeld = false;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);
        Logger.recordOutput("intakeCurrent", inputs.motorCurrent);

        noteThere = inputs.motorCurrent > 12;

        // turns motors off if button isn't held and note isn't detected
        if (!buttonHeld && !noteThere) {
            volts = 0;
        }

        io.setVoltage(volts);
    }

    public Command getIntakeCommand(double v) {
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteThere(),
                this);
    }
}