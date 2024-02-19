package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

// manages logging data and motor
public class IntakeIOFalcon implements IntakeIO {
    TalonFX motor, follower;

    StatusSignal<Double> voltageSignal = motor.getMotorVoltage();
    StatusSignal<Double> currentSignal = motor.getSupplyCurrent();
    StatusSignal<Double> velocitySignal = motor.getVelocity();
    StatusSignal<Double> tempSignal = motor.getDeviceTemp();

    public IntakeIOFalcon(int port1, int port2) {
        motor = new TalonFX(port1);
        follower = new TalonFX(port2);
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
        follower.setVoltage(volts); // as far as I can find, there is no .follow() method for TalonFX
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, tempSignal);
        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorTemperature = tempSignal.getValue();
    }
}