package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private Measure<Velocity<Angle>> leftVelocity = RotationsPerSecond.of(0);
    private Measure<Velocity<Angle>> rightVelocity = RotationsPerSecond.of(0);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
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
        return inputs.leftShooterCurrent > 30;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("Shooter on", leftVelocity.magnitude() != 0 || rightVelocity.magnitude() != 0);

        // io.setVelocity(leftVelocity, rightVelocity);
        // io.setVoltage(Volts.of(6));
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
}