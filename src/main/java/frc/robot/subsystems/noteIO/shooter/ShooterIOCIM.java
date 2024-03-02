package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;

public class ShooterIOCIM implements ShooterIO {
    TalonSRX motor1, motor2;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);

    public ShooterIOCIM(int id1, int id2) {
        motor1 = new TalonSRX(id1);
        motor2 = new TalonSRX(id2);
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        double feedForwardVolts = feedForward.calculate(velocity.in(RotationsPerSecond));
        double pidVolts = pid.calculate(motor1.getSelectedSensorVelocity(), velocity.in(RotationsPerSecond));
        setVoltage(Volts.of(feedForwardVolts + pidVolts));
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        motor1.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
        motor2.set(TalonSRXControlMode.PercentOutput, volts.in(Volts) / 12);
    }

    @Override
    public boolean upToSpeed(Measure<Velocity<Angle>> targetVelocity) {
        return motor1.getSelectedSensorVelocity() * 10 > targetVelocity.in(RotationsPerSecond) * 0.95;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motor1Voltage = motor1.getMotorOutputVoltage();
        inputs.motor1Current = motor1.getSupplyCurrent();
        inputs.motor1Position = motor1.getSelectedSensorPosition();
        inputs.motor1Velocity = motor1.getSelectedSensorVelocity() * 10; // function records rotations per 100ms
        inputs.motor1Temperature = motor1.getTemperature();

        inputs.motor2Voltage = motor2.getMotorOutputVoltage();
        inputs.motor2Current = motor2.getSupplyCurrent();
        inputs.motor2Position = motor2.getSelectedSensorPosition();
        inputs.motor2Velocity = motor2.getSelectedSensorVelocity() * 10; // function records rotations per 100ms
        inputs.motor2Temperature = motor2.getTemperature();
    }
}