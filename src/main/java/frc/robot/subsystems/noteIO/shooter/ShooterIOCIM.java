package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.HardwareConstants;

public class ShooterIOCIM implements ShooterIO {
    TalonSRX motor1, motor2;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);
    Debouncer currentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

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
        // returns whether current has risen for more than 0.1 seconds
        return motor1.getSelectedSensorVelocity() * 10 * 60 > (HardwareConstants.CIM_MAX_RPM / 60)
                        * (motor1.getMotorOutputVoltage() / 12);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.motorVoltage = motor1.getMotorOutputVoltage();
        inputs.motorCurrent = motor1.getSupplyCurrent();
        inputs.motorVelocity = motor1.getSelectedSensorVelocity() * 10 * 60; // function records rotations per 100ms
        inputs.motorTemperature = motor1.getTemperature();
    }
}