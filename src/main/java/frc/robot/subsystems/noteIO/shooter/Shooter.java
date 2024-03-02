package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    double velocity;
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

    public void start(double v) {
        this.velocity = v;
        time = Timer.getFPGATimestamp();
    }

    public void stop() {
        this.velocity = 0;
    }

    public boolean upToSpeed() {
        return io.upToSpeed();
    }

    public boolean checkNoteShot() {
        // returns whether note detected and it's been 2 seconds
        return inputs.motor1Current > 30 && Timer.getFPGATimestamp() - time > 2;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        // io.setVelocity(velocity);
        io.setVoltage(velocity);
    }
}