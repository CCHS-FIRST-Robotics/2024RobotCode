package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class ShooterIOFalcon implements ShooterIO {
    TalonFX motor;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);

    StatusSignal<Double> voltageSignal = motor.getMotorVoltage();
    StatusSignal<Double> currentSignal = motor.getSupplyCurrent();
    StatusSignal<Double> velocitySignal = motor.getVelocity();
    StatusSignal<Double> tempSignal = motor.getDeviceTemp();

    public ShooterIOFalcon(int port) {
        motor = new TalonFX(port);
    }

    @Override
    public void setVelocity(double velocity) {
        double feedForwardVolts = feedForward.calculate(velocity);
        double pidVolts = pid.calculate(velocitySignal.refresh().getValue(), velocity);
        this.setVoltage(feedForwardVolts + pidVolts);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, tempSignal);
        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
    }
}