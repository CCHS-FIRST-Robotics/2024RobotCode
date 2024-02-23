package frc.robot.subsystems.noteIO.intakeGround;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.subsystems.noteIO.intakeArm.IntakeArmIO;

import com.revrobotics.RelativeEncoder;

public class IntakeIONEO implements IntakeArmIO {
    CANSparkMax motor1, motor2;
    RelativeEncoder encoder1, encoder2;

    public IntakeIONEO(int id) {
        motor1 = new CANSparkMax(id, MotorType.kBrushless);
        encoder1 = motor1.getEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeGroundIOInputsAutoLogged inputs) {
        inputs.motorVoltage = motor1.getBusVoltage();
        inputs.motorCurrent = motor1.getOutputCurrent();
        inputs.motorVelocity = encoder1.getVelocity();
        inputs.motorTemperature = motor1.getMotorTemperature();
    }
}