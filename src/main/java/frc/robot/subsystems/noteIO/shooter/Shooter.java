package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private Measure<Velocity<Angle>> leftVelocity = RotationsPerSecond.of(0);
    private Measure<Velocity<Angle>> rightVelocity = RotationsPerSecond.of(0);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    double time;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(Measure<Velocity<Angle>> velocity) {
        this.leftVelocity = velocity;
        this.rightVelocity = velocity;
        time = Timer.getFPGATimestamp();
    }

    public void start(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        time = Timer.getFPGATimestamp();
    }

    public void stop() {
        leftVelocity = RotationsPerSecond.of(0);
        rightVelocity = RotationsPerSecond.of(0);
    }

    @AutoLogOutput
    public boolean upToSpeed() {
        return io.upToSpeed(leftVelocity, rightVelocity);
    }

    @AutoLogOutput
    public boolean checkNoteShot() {
        return inputs.rightShooterCurrent > 30 && Timer.getFPGATimestamp() - time > 1;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("Shooter on", leftVelocity.in(RadiansPerSecond) != 0 || rightVelocity.in(RadiansPerSecond) != 0);

        io.setVelocity(leftVelocity, rightVelocity);
        // io.setVoltage(Volts.of(1));
    }

    @AutoLogOutput
    public boolean isOn() {
        return rightVelocity.in(RotationsPerSecond) != 0;
    }

    public Command getShootNoteCommand(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        // turns on motor to shoot note
        return new FunctionalCommand(
                () -> start(leftVelocity, rightVelocity),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteShot(),
                this);
    }

    public void addToOrchestra(Orchestra orchestra, int trackNum) {
        io.addToOrchestra(orchestra, trackNum);
    }
}