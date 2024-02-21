package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class ShooterIOCIM implements ShooterIO {
    TalonSRX motor;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);

    public ShooterIOCIM(int port) {
        motor = new TalonSRX(port);
    }

    @Override
    public void setVelocity(double velocity) {
        double feedForwardVolts = feedForward.calculate(velocity);
        double pidVolts = pid.calculate(motor.getSelectedSensorVelocity(), velocity);
        this.setVoltage(feedForwardVolts + pidVolts);
    }

    @Override
    public void setVoltage(double volts) {
        motor.set(TalonSRXControlMode.PercentOutput, volts / 12);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.motorVoltage = motor.getMotorOutputVoltage();
        inputs.motorCurrent = motor.getSupplyCurrent();
        inputs.motorVelocity = motor.getSelectedSensorVelocity();
    }
}