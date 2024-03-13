package frc.robot.subsystems.noteIO.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.*;

import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeIONEO implements IntakeIO {
    CANSparkMax intake1, intake2;
    RelativeEncoder encoder1, encoder2;

    public IntakeIONEO(int id1, int id2) {
        intake1 = new CANSparkMax(id1, MotorType.kBrushless);
        intake2 = new CANSparkMax(id1, MotorType.kBrushless);
        encoder1 = intake1.getEncoder();
        encoder2 = intake2.getEncoder(); // should this be motor 2?

        intake1.setCANTimeout(500);
        intake2.setCANTimeout(500);
        intake1.setInverted(true);
        intake2.setInverted(false);
        intake1.setSmartCurrentLimit(30);
        intake2.setSmartCurrentLimit(30);
        intake1.setIdleMode(IdleMode.kCoast);
        intake2.setIdleMode(IdleMode.kCoast);
        intake1.setCANTimeout(0);
        intake2.setCANTimeout(0);

        System.out.println("SM burn flash:");
        System.out.println(intake1.burnFlash() == REVLibError.kOk);
        System.out.println(intake2.burnFlash() == REVLibError.kOk);
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        intake1.setVoltage(v.in(Volts));
        intake2.setVoltage(v.in(Volts));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motor1Voltage = intake1.getBusVoltage();
        inputs.motor1Current = intake1.getOutputCurrent();
        inputs.motor1Velocity = encoder1.getVelocity();
        inputs.motor1Temperature = intake1.getMotorTemperature();

        inputs.motor2Voltage = intake2.getBusVoltage();
        inputs.motor2Current = intake2.getOutputCurrent();
        inputs.motor2Velocity = encoder2.getVelocity();
        inputs.motor2Temperature = intake2.getMotorTemperature();
    }
}