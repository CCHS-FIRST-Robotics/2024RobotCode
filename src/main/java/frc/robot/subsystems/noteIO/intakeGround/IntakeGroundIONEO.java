package frc.robot.subsystems.noteIO.intakeGround;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class IntakeGroundIONEO implements IntakeGroundIO {
    CANSparkMax motor1, motor2;
    RelativeEncoder encoder1, encoder2;

    public IntakeGroundIONEO(int id) {
        motor1 = new CANSparkMax(id, MotorType.kBrushless);
        encoder1 = motor1.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeGroundIOInputsAutoLogged inputs) {
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