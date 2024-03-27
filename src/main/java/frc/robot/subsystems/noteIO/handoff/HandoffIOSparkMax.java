package frc.robot.subsystems.noteIO.handoff;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;

public class HandoffIOSparkMax implements HandoffIO {
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    public HandoffIOSparkMax(int id) {
        motor = new CANSparkMax(id, CANSparkBase.MotorType.kBrushless);
        encoder = motor.getEncoder(); 

        motor.setCANTimeout(500);
        motor.setInverted(false);
        motor.setSmartCurrentLimit(60);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setCANTimeout(0);

        System.out.println("SM burn flash:");
        System.out.println(motor.burnFlash() == REVLibError.kOk);
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        motor.setVoltage(v.in(Volts));
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        inputs.motorVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
        inputs.motorTemperature = motor.getMotorTemperature();
    }
}