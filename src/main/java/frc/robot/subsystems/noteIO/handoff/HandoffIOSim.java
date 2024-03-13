package frc.robot.subsystems.noteIO.handoff;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HandoffIOSim implements HandoffIO {
    Measure<Voltage> appliedVoltage = Volts.of(0);
    DCMotorSim motor = new DCMotorSim(DCMotor.getFalcon500(1), 1, .001);

    public HandoffIOSim() {

    }

    public void updateInputs(HandoffIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.motorVoltage = appliedVoltage.in(Volts);
        inputs.motorCurrent = motor.getCurrentDrawAmps();
        inputs.motorVelocity = motor.getAngularVelocityRPM();
    }

    public void setVoltage(double volts) {
        appliedVoltage = Volts.of(volts);
        motor.setInputVoltage(volts);
    }
}