package frc.robot.subsystems.noteIO.intakeArm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class IntakeArmIOFalcon500 implements IntakeArmIO {
    TalonFX motor;

    StatusSignal<Double> voltageSignal;
    StatusSignal<Double> currentSignal;
    StatusSignal<Double> velocitySignal;
    StatusSignal<Double> temperatureSignal;

    public IntakeArmIOFalcon500(int id) {
        motor = new TalonFX(id);

        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getSupplyCurrent();
        velocitySignal = motor.getVelocity();
        temperatureSignal = motor.getDeviceTemp();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeArmIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorTemperature = temperatureSignal.getValue();
    }
}