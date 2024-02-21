package frc.robot.subsystems.noteIO.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class IntakeIONeo implements IntakeIO {
    CANSparkMax motor;
    RelativeEncoder encoder;

    public IntakeIONeo(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorVelocity = encoder.getVelocity();
    }
}