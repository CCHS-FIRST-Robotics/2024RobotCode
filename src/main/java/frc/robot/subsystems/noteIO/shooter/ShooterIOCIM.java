package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class ShooterIOCIM implements ShooterIO {
    TalonSRX motor1, motor2;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);

    public ShooterIOCIM(int id1, int id2) {
        motor1 = new TalonSRX(id1);
        motor2 = new TalonSRX(id2);
    }

    @Override
    public void setVelocity(double velocity) {
        double feedForwardVolts = feedForward.calculate(velocity);
        double pidVolts = pid.calculate(motor1.getSelectedSensorVelocity(), velocity);
        setVoltage(feedForwardVolts + pidVolts);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts / 12);
        motor2.set(TalonSRXControlMode.PercentOutput, volts / 12);
    }

    @Override
    public boolean upToSpeed() {
        return motor1.getSelectedSensorVelocity() * 10 * 60 > 98 * (0.5);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motor1Voltage = motor1.getMotorOutputVoltage();
        inputs.motor1Current = motor1.getSupplyCurrent();
        inputs.motor1Position = motor1.getSelectedSensorPosition();
        inputs.motor1Velocity = motor1.getSelectedSensorVelocity() * 10 * 60; // function records rotations per 100ms
        inputs.motor1Temperature = motor1.getTemperature();

        inputs.motor2Voltage = motor2.getMotorOutputVoltage();
        inputs.motor2Current = motor2.getSupplyCurrent();
        inputs.motor2Position = motor2.getSelectedSensorPosition();
        inputs.motor2Velocity = motor2.getSelectedSensorVelocity() * 10 * 60; // function records rotations per 100ms
        inputs.motor2Temperature = motor2.getTemperature();
    }
}