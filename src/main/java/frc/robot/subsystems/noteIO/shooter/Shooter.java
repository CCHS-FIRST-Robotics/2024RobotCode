package frc.robot.subsystems.noteIO.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    boolean upToSpeed = false;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void shoot(double velocity) {
        io.setVelocity(velocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("shooterCurrent", inputs.motorCurrent);

        if (inputs.motorCurrent > 12 && !upToSpeed) {
            upToSpeed = true;
        }

        // turn motors off after note is no longer being detected
        if (inputs.motorCurrent < 12 && upToSpeed) {
            io.setVoltage(0);
            upToSpeed = false;
        }
    }
}