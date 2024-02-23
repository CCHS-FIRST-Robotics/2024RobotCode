package frc.robot.subsystems.noteIO.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(double velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.setVelocity(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }

    public boolean checkCompleteShot() {
        // kinda placeholder thing idk if shooter should always run for 5s or something to detect if its gone 
        return inputs.motorCurrent > 15 && inputs.motorVelocity > (4000 / 60d) * (inputs.motorVoltage / 12d);
    }

    public boolean checkInHandoff() {
        // also placeholder, i think could use current limits with the handoff motor idk
        return false;
    }

    public Command getShootNoteCommand(double v) {
        // turns on motor to shoot note
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkCompleteShot(),
                this);
    }

    public Command getReceiveNoteCommand(double v) {
        // turns on motor until note is fully detected inside handoff
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkInHandoff(),
                this);
    }
}