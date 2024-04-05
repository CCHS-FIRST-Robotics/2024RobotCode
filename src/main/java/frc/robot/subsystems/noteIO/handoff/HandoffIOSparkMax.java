package frc.robot.subsystems.noteIO.handoff;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.*;

public class HandoffIOSparkMax implements HandoffIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    public HandoffIOSparkMax(int id) {
        motor = new CANSparkMax(id, CANSparkBase.MotorType.kBrushless);
        encoder = motor.getEncoder();

        // motor setup
        motor.setCANTimeout(500);
        motor.setInverted(false);
        motor.setSmartCurrentLimit(60);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setCANTimeout(0);
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        motor.setVoltage(v.in(Volts));
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        inputs.motorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
        inputs.motorTemperature = motor.getMotorTemperature();
    }
}