package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    SysIdRoutine sysIdRoutine;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            io.setVoltage(volts.in(Volts));
                        },
                        null,
                        this));
    }

    public void start(double velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.setVelocity(0);
    }

    public boolean checkNoteShot() {
        return io.checkNoteShot();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }
}