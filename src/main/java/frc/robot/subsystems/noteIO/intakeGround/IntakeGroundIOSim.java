package frc.robot.subsystems.noteIO.intakeGround;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeGroundIOSim implements IntakeGroundIO {
    Measure<Voltage> appliedVoltage = Volts.of(0);
    DCMotorSim motor1 = new DCMotorSim(DCMotor.getFalcon500(1), 1, .001);
    DCMotorSim motor2 = new DCMotorSim(DCMotor.getFalcon500(1), 1, .001);

    public IntakeGroundIOSim() {

    }

    public void updateInputs(IntakeGroundIOInputs inputs) {
        motor1.update(Constants.PERIOD);

        inputs.motor1Voltage = appliedVoltage.in(Volts);
        inputs.motor1Current = motor1.getCurrentDrawAmps();
        inputs.motor1Velocity = motor1.getAngularVelocityRPM();

        motor2.update(Constants.PERIOD);

        inputs.motor2Voltage = appliedVoltage.in(Volts);
        inputs.motor2Current = motor2.getCurrentDrawAmps();
        inputs.motor2Velocity = motor2.getAngularVelocityRPM();
    }

    public void setVoltage(double volts) {
        appliedVoltage = Volts.of(volts);
        motor1.setInputVoltage(volts);
        motor2.setInputVoltage(volts);
    }
}