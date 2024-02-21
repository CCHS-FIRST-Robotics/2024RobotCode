package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class IntakeIOFalcon implements IntakeIO {
    TalonFX motor1, motor2;

    StatusSignal<Double> voltageSignal1;
    StatusSignal<Double> currentSignal1;
    StatusSignal<Double> velocitySignal1;
    StatusSignal<Double> tempSignal1;

    StatusSignal<Double> voltageSignal2;
    StatusSignal<Double> currentSignal2;
    StatusSignal<Double> velocitySignal2;
    StatusSignal<Double> tempSignal2;
    

    public IntakeIOFalcon(int port1, int port2) {
        motor1 = new TalonFX(port1);
        motor2 = new TalonFX(port2);

        voltageSignal1 = motor1.getMotorVoltage();
        currentSignal1 = motor1.getSupplyCurrent();
        velocitySignal1 = motor1.getVelocity();
        tempSignal1 = motor1.getDeviceTemp();

        voltageSignal2 = motor2.getMotorVoltage();
        currentSignal2 = motor2.getSupplyCurrent();
        velocitySignal2 = motor2.getVelocity();
        tempSignal2 = motor2.getDeviceTemp();
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
        motor2.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal1, currentSignal1, velocitySignal1,
                voltageSignal2, currentSignal2, velocitySignal2);

        inputs.motor1Voltage = voltageSignal1.getValue();
        inputs.motor1Current = currentSignal1.getValue();
        inputs.motor1Velocity = velocitySignal1.getValue();

        inputs.motor2Voltage = voltageSignal2.getValue();
        inputs.motor2Current = currentSignal2.getValue();
        inputs.motor2Velocity = velocitySignal2.getValue();
    }
}