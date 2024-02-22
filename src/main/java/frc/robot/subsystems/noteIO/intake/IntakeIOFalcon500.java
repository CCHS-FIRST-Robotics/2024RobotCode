package frc.robot.subsystems.noteIO.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class IntakeIOFalcon500 implements IntakeIO {
    TalonFX motor;

    StatusSignal<Double> voltageSignal = motor.getMotorVoltage();
    StatusSignal<Double> currentSignal = motor.getSupplyCurrent();
    StatusSignal<Double> velocitySignal = motor.getVelocity();
    StatusSignal<Double> temperatureSignal = motor.getDeviceTemp();

    public IntakeIOFalcon500(int id) {
        motor = new TalonFX(id);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorTemperature = temperatureSignal.getValue();
    }
}