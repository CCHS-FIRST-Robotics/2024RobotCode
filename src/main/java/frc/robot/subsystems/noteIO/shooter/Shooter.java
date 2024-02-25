package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.*;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    SysIdRoutine sysIdRoutine;
    Debouncer currentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    // double startTime;
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
        // startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        io.setVelocity(0);
    }

    public boolean checkNoteShot() {
        // returns whether current has risen for more than 0.1 seconds
        return currentDebouncer.calculate(inputs.motorCurrent > 15);

        // // returns whether note friction detected and motor up to speed
        // return inputs.motorCurrent > 15
        // && inputs.motorVelocity > (maxRPM / 60) * (inputs.motorVoltage / 12);

        // // returns if 4 seconds have gone by
        // return Timer.getFPGATimestamp() - startTime > 4000;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }
}