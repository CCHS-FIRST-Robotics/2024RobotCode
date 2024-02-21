package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class ShooterIOFalcon implements ShooterIO {
    TalonFX motor1, motor2;

    // I have no idea how to do a motion magic
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);

    MotionMagicVoltage driveMotionMagic = new MotionMagicVoltage(0);

    StatusSignal<Double> voltageSignal = motor1.getMotorVoltage();
    StatusSignal<Double> currentSignal = motor1.getSupplyCurrent();
    StatusSignal<Double> velocitySignal = motor1.getVelocity();
    StatusSignal<Double> tempSignal = motor1.getDeviceTemp();

    public ShooterIOFalcon(int id1, int id2) {
        motor1 = new TalonFX(id1);
        motor2 = new TalonFX(id2);
    }

    @Override
    public void setVelocity(double velocity) {
        double feedForwardVolts = feedForward.calculate(velocity);
        double pidVolts = pid.calculate(velocitySignal.refresh().getValue(), velocity);

        // ! might be right????
        motor1.setControl(driveMotionMagic.withFeedForward(feedForwardVolts));

        setVoltage(feedForwardVolts + pidVolts);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
        motor2.setVoltage(volts);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal);
        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
    }
}