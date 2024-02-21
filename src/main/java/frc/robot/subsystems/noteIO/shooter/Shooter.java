package frc.robot.subsystems.noteIO.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

// primeShooter spins shotoer up to speed
// shoot moves the note to the shooter
// 2 falcons
public class Shooter extends SubsystemBase {
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void start(double velocity) {
        io.setVelocity(velocity);
    }

    public void stop() {
        io.setVelocity(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("shooterCurrent", inputs.motorCurrent);
        // why are you logging a subfield of inputs if youre logging inputs
    }
}