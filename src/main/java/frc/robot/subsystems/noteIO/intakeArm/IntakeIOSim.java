package frc.robot.subsystems.noteIO.intakeArm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeArmIO {
    Measure<Voltage> appliedVoltage = Volts.of(0);
    DCMotorSim motor = new DCMotorSim(DCMotor.getFalcon500(1), 1, .001);

    public IntakeIOSim() {

    }

    public void updateInputs(IntakeArmIOInputs inputs) {
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