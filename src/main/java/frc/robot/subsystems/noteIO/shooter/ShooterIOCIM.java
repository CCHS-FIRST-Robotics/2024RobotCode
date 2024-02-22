package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
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
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.motorVoltage = motor1.getMotorOutputVoltage();
        inputs.motorCurrent = motor1.getSupplyCurrent();
        inputs.motorVelocity = motor1.getSelectedSensorVelocity();
        inputs.motorTemperature = motor1.getTemperature();
    }
}