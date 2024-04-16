package frc.robot.subsystems.noteIO.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.*;

public class IntakeIONEO implements IntakeIO {
    CANSparkMax motor1, motor2;
    RelativeEncoder encoder1, encoder2;

    public IntakeIONEO(int id1, int id2) {
        motor1 = new CANSparkMax(id1, MotorType.kBrushless);
        motor2 = new CANSparkMax(id2, MotorType.kBrushless);
        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        // motor setup
        motor1.setCANTimeout(500);
        motor2.setCANTimeout(500);
        motor1.setInverted(false);
        motor2.setInverted(true);
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor1.setIdleMode(IdleMode.kCoast);
        motor2.setIdleMode(IdleMode.kCoast);
        motor1.setCANTimeout(0);
        motor2.setCANTimeout(0);
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        motor1.setVoltage(v.in(Volts));
        motor2.setVoltage(v.in(Volts));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
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