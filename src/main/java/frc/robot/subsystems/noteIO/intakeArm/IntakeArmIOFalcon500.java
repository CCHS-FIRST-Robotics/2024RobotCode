package frc.robot.subsystems.noteIO.intakeArm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class IntakeArmIOFalcon500 implements IntakeArmIO {
    private TalonFX motor;

    private StatusSignal<Double> voltageSignal;
    private StatusSignal<Double> currentSignal;
    private StatusSignal<Double> positionSignal;
    private StatusSignal<Double> velocitySignal;
    private StatusSignal<Double> temperatureSignal;

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
        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
        motor.getConfigurator().apply(talonFXConfig);
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        motor.setVoltage(v.in(Volts));
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