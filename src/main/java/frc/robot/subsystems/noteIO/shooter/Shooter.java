package frc.robot.subsystems.noteIO.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    ShooterIO io;
    int maxRPM;
    // double startTime;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io, int maxRPM) {
        this.io = io;
        this.maxRPM = maxRPM;
    }

    public void start(double velocity) {
        io.setVelocity(velocity);
        // startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        io.setVelocity(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }

    public boolean checkNoteShot() {
        // returns if note friction detected and motor up to speed
        return inputs.motorCurrent > 15
                && inputs.motorVelocity > (maxRPM / 60) * (inputs.motorVoltage / 12);

        // returns if 4 seconds have gone by
        // return Timer.getFPGATimestamp() - startTime > 4000;
    }
}