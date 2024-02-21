package frc.robot.subsystems.noteIO.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class IntakeIOFalcon implements IntakeIO {
    TalonFX motor, follower;

    StatusSignal<Double> voltageSignal = motor.getMotorVoltage();
    StatusSignal<Double> currentSignal = motor.getSupplyCurrent();
    StatusSignal<Double> velocitySignal = motor.getVelocity();
    StatusSignal<Double> tempSignal = motor.getDeviceTemp();

    public IntakeIOFalcon(int port) {
        motor = new TalonFX(port);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, tempSignal);
        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorTemperature = tempSignal.getValue();
    }
}