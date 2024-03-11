package frc.robot.subsystems.noteIO.intakeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class IntakeBaseIONEO implements IntakeBaseIO {
    CANSparkMax motor1, motor2;
    RelativeEncoder encoder1, encoder2;

    public IntakeBaseIONEO(int id1, int id2) {
        motor1 = new CANSparkMax(id1, MotorType.kBrushless);
        encoder1 = motor1.getEncoder();
        motor2 = new CANSparkMax(id1, MotorType.kBrushless);
        encoder2 = motor2.getEncoder(); //should this be motor 2?

        encoder1.setVelocityConversionFactor(1 / 60);
        encoder2.setVelocityConversionFactor(1 / 60);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
        motor2.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeBaseIOInputsAutoLogged inputs) {
        inputs.motor1Voltage = motor1.getBusVoltage();
        inputs.motor1Current = motor1.getOutputCurrent();
        inputs.motor1Velocity = encoder1.getVelocity();
        inputs.motor1Temperature = motor1.getMotorTemperature();

        inputs.motor2Voltage = motor2.getBusVoltage();
        inputs.motor2Current = motor2.getOutputCurrent();
        inputs.motor2Velocity = encoder2.getVelocity();
        inputs.motor2Temperature = motor2.getMotorTemperature();
    }
}