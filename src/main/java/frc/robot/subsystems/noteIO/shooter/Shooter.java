package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoPathConstants;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private Measure<Velocity<Angle>> leftVelocity = RotationsPerSecond.of(0);
    private Measure<Velocity<Angle>> rightVelocity = RotationsPerSecond.of(0);
    private double startTime;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    boolean gottenToSpeed;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        gottenToSpeed = false;
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
        return inputs.leftMotorCurrent > 18 && gottenToSpeed;
        // return Timer.getFPGATimestamp() - startTime > AutoPathConstants.Q_SHOOT_TIME;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("Shooting", leftVelocity.magnitude() != 0 || rightVelocity.magnitude() != 0);

        if (upToSpeed()) {
            gottenToSpeed = true;
        }

        io.setVelocity(leftVelocity, rightVelocity);
    }

    @AutoLogOutput
    public boolean checkInHandoff() {
        return Timer.getFPGATimestamp() - startTime > 0.1;
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

    public Command getReceiveNoteCommand(Measure<Velocity<Angle>> leftVelocity,
            Measure<Velocity<Angle>> rightVelocity) {
        // turns on motor until note is fully detected inside handoff
        return new FunctionalCommand(
                () -> start(leftVelocity, rightVelocity),
                () -> {
                },
                (interrupted) -> stop(),
                () -> checkInHandoff(),
                this);
    }
}