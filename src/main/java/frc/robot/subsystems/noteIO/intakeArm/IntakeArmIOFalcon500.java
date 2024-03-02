package frc.robot.subsystems.noteIO.intakeArm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeArmIOFalcon500 implements IntakeArmIO {
    TalonFX motor;

    StatusSignal<Double> voltageSignal;
    StatusSignal<Double> currentSignal;
    StatusSignal<Double> positionSignal;
    StatusSignal<Double> velocitySignal;
    StatusSignal<Double> temperatureSignal;

    public IntakeArmIOFalcon500(int id) {
        motor = new TalonFX(id);

        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        currentSignal.setUpdateFrequency(100);
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        temperatureSignal = motor.getDeviceTemp();

        // current limiting
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 60;
        motor.getConfigurator().apply(talonFXConfig);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, positionSignal, velocitySignal, temperatureSignal);

        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorPosition = positionSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorTemperature = temperatureSignal.getValue();
    }
}