package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    SysIdRoutine sysIdRoutine;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    double time;

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
        // for testing
        io.setVoltage(6);
        time = Timer.getFPGATimestamp();
    }

    public void stop() {
        // io.setVelocity(0);
        // for testing
        io.setVoltage(0);
    }

    @AutoLogOutput
    public boolean upToSpeed(){
        // returns whether current has risen for more than 0.1 seconds
        return inputs.motorVelocity > 95
                        * (.5);
    }

    public boolean checkNoteShot() {
        return inputs.motorCurrent > 30 && Timer.getFPGATimestamp() - time > 2;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }
}