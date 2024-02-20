package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class IntakeIOCIM implements IntakeIO {
    TalonSRX motor1, motor2;

    public IntakeIOCIM(int port1, int port2) {
        motor1 = new TalonSRX(port1);
        motor2 = new TalonSRX(port2);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts / 12);
        motor2.set(TalonSRXControlMode.PercentOutput, volts / 12);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motor1Voltage = motor1.getMotorOutputVoltage();
        inputs.motor1Current = motor1.getSupplyCurrent();
        inputs.motor1Velocity = motor1.getSelectedSensorVelocity();

        inputs.motor2Voltage = motor2.getMotorOutputVoltage();
        inputs.motor2Current = motor2.getSupplyCurrent();
        inputs.motor2Velocity = motor2.getSelectedSensorVelocity();
    }
}