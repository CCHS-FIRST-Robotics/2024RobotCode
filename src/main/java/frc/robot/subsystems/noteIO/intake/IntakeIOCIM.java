package frc.robot.subsystems.noteIO.intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class IntakeIOCIM implements IntakeIO {
    TalonSRX motor, follower;

    public IntakeIOCIM(int port1, int port2) {
        motor = new TalonSRX(port1);
        follower = new TalonSRX(port2);
        follower.follow(motor);
    }

    @Override
    public void setVoltage(double volts) {
        motor.set(TalonSRXControlMode.PercentOutput, volts / 12);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorOutputVoltage();
        inputs.motorCurrent = motor.getSupplyCurrent();
        inputs.motorVelocity = motor.getSelectedSensorVelocity();
        inputs.motorTemperature = motor.getTemperature();
    }
}