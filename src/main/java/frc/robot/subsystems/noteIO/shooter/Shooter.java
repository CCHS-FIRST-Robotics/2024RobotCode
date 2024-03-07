package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoPathConstants;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private Measure<Velocity<Angle>> velocity = RotationsPerSecond.of(0);
    private double startTime;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(Measure<Velocity<Angle>> v) {
        velocity = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        velocity = RotationsPerSecond.of(0);
    }

    public boolean upToSpeed() {
        return io.upToSpeed(velocity);
    }

    public boolean checkNoteShot() {
        // return inputs.motor1Current > 18 && Timer.getFPGATimestamp() - startTime > 2;
        return Timer.getFPGATimestamp() - startTime > AutoPathConstants.Q_SHOOT_TIME;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("Shooting", velocity.magnitude() != 0);

        io.setVelocity(velocity);
    }

    public boolean checkInHandoff() {
        return Timer.getFPGATimestamp() - startTime > 0.1;
    }

    public Command getShootNoteCommand(Measure<Velocity<Angle>> v) {
        // turns on motor to shoot note
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteShot(),
                this);
    }

    public Command getReceiveNoteCommand(Measure<Velocity<Angle>> v) {
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