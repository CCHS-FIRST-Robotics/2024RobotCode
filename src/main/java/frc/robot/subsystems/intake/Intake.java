package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerveDrive.ModuleIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    IntakeIO io;
    double volts = 0;
    double startTime; // temp
    // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void start(double v) {
        volts = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = 0;
    }

    @Override
    public void periodic() {
        // io.updateInputs(inputs);
        // Logger.processInputs("intake", inputs);

        // for when we have an encoder:
        // if(inputs.motor1Current > 15 && inputs.motor1Velocity > 5000 * (volts / 12)){
        // if (inputs.motor1Current > 15 && Timer.getFPGATimestamp() - startTime > 0.4) {
        //     volts = 0;
        // }

        Logger.recordOutput("Intaking", volts != 0);

        io.setVoltage(volts);
    }
}