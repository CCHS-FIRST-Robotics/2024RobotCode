package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private Measure<Velocity<Angle>> leftVelocity = RotationsPerSecond.of(0);
    private Measure<Velocity<Angle>> rightVelocity = RotationsPerSecond.of(0);
    double startTime;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        leftVelocity = RotationsPerSecond.of(0);
        rightVelocity = RotationsPerSecond.of(0);
    }

    public boolean upToSpeed() {
        return io.upToSpeed(leftVelocity, rightVelocity);
    }

    public boolean checkNoteShot() {
        return inputs.rightShooterCurrent > 30 && Timer.getFPGATimestamp() - startTime > 1;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter on",
                leftVelocity.in(RadiansPerSecond) + rightVelocity.in(RadiansPerSecond) != 0);

        io.setVelocity(leftVelocity, rightVelocity);
    }

    // turns shooter on
    // ! unused, see if we can implement it
    public Command getShootNoteCommand(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        return new FunctionalCommand(
                () -> start(leftVelocity, rightVelocity),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkNoteShot(),
                this);
    }
}