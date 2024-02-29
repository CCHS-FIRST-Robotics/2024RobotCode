package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    double time;
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
        // io.setVelocity(velocity);

        io.setVoltage(velocity);
        time = Timer.getFPGATimestamp();
    }

    public void stop() {
        // io.setVelocity(0);

        io.setVoltage(0);
    }

    public boolean upToSpeed() {
        return inputs.motorVelocity > 98 * (0.5);
    }

    public boolean checkNoteShot() {
        // returns whether note detected and it's been 2 seconds
        return inputs.motorCurrent > 30 && Timer.getFPGATimestamp() - time > 2;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }
}