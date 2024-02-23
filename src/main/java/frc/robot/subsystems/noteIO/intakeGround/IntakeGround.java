package frc.robot.subsystems.noteIO.intakeGround;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeGround extends SubsystemBase {
    IntakeGroundIO io;
    double volts = 0;
    IntakeGroundIOInputsAutoLogged inputs = new IntakeGroundIOInputsAutoLogged();

    public IntakeGround(IntakeGroundIO io) {
        this.io = io;
    }

    public void start(double v) {
        volts = v;
    }

    public void stop() {
        volts = 0;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        if (inputs.motor1Current > 15 && inputs.motor1Velocity > 5000 * (volts / 12)) {
            volts = 0;
        }

        io.setVoltage(volts);
    }
}