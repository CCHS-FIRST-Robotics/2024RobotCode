package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class IntakeIOCIM implements IntakeIO {
    TalonSRX motor, follower;

    public IntakeIOCIM(int port1, int port2) {
        motor = new TalonSRX(port1);
        follower = new TalonSRX(port2);
        follower.follow(motor);
    }

    public void setVoltage(double volts) {
        motor.set(TalonSRXControlMode.PercentOutput, volts / 12);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorCurrent = motor.getSupplyCurrent();
        inputs.motorVoltage = motor.getMotorOutputVoltage();
        inputs.motorVelocity = motor.getSelectedSensorVelocity();
        inputs.motorTemperature = motor.getTemperature();
    }
}