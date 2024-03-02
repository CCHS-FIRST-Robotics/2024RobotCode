package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterIOFalcon500 implements ShooterIO {
    TalonFX motor1, motor2;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);

    StatusSignal<Double> voltageSignal1;
    StatusSignal<Double> currentSignal1;
    StatusSignal<Double> positionSignal1;
    StatusSignal<Double> velocitySignal1;
    StatusSignal<Double> temperatureSignal1;

    StatusSignal<Double> voltageSignal2;
    StatusSignal<Double> currentSignal2;
    StatusSignal<Double> positionSignal2;
    StatusSignal<Double> velocitySignal2;
    StatusSignal<Double> temperatureSignal2;

    public ShooterIOFalcon500(int id1, int id2) {
        motor1 = new TalonFX(id1);
        motor2 = new TalonFX(id2);

        voltageSignal1 = motor1.getMotorVoltage();
        currentSignal1 = motor1.getStatorCurrent();
        currentSignal1.setUpdateFrequency(100);
        positionSignal1 = motor1.getPosition();
        velocitySignal1 = motor1.getVelocity();
        temperatureSignal1 = motor1.getDeviceTemp();

        voltageSignal2 = motor2.getMotorVoltage();
        currentSignal2 = motor2.getStatorCurrent();
        currentSignal2.setUpdateFrequency(100);
        positionSignal2 = motor2.getPosition();
        velocitySignal2 = motor2.getVelocity();
        temperatureSignal2 = motor2.getDeviceTemp();

        // current limiting
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 60;
        motor1.getConfigurator().apply(talonFXConfig);
        motor2.getConfigurator().apply(talonFXConfig);
    }

    @Override
    public void setVelocity(double velocity) {
        double feedForwardVolts = feedForward.calculate(velocity);
        double pidVolts = pid.calculate(velocitySignal1.refresh().getValue(), velocity);
        setVoltage(feedForwardVolts + pidVolts);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
        motor2.setVoltage(volts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                voltageSignal1,
                currentSignal1,
                positionSignal1,
                velocitySignal1,
                temperatureSignal1,
                voltageSignal2,
                currentSignal2,
                positionSignal2,
                velocitySignal2,
                temperatureSignal2);

        inputs.motor1Voltage = voltageSignal1.getValue();
        inputs.motor1Current = currentSignal1.getValue();
        inputs.motor1Position = positionSignal1.getValue();
        inputs.motor1Velocity = velocitySignal1.getValue();
        inputs.motor1Temperature = temperatureSignal1.getValue();

        inputs.motor2Voltage = voltageSignal2.getValue();
        inputs.motor2Current = currentSignal2.getValue();
        inputs.motor2Position = positionSignal2.getValue();
        inputs.motor2Velocity = velocitySignal2.getValue();
        inputs.motor2Temperature = temperatureSignal2.getValue();
    }
}