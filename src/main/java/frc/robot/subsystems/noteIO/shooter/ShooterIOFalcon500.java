package frc.robot.subsystems.noteIO.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.HardwareConstants;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

public class ShooterIOFalcon500 implements ShooterIO {
    TalonFX motor1, motor2;
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);
    PIDController pid = new PIDController(0, 0, 0);
    Debouncer currentDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    StatusSignal<Double> voltageSignal;
    StatusSignal<Double> currentSignal;
    StatusSignal<Double> velocitySignal;
    StatusSignal<Double> temperatureSignal;

    public ShooterIOFalcon500(int id1, int id2) {
        motor1 = new TalonFX(id1);
        motor2 = new TalonFX(id2);

        voltageSignal = motor1.getMotorVoltage();
        currentSignal = motor1.getSupplyCurrent();
        velocitySignal = motor1.getVelocity();
        temperatureSignal = motor1.getDeviceTemp();
    }

    @Override
    public void setVelocity(double velocity) {
        double feedForwardVolts = feedForward.calculate(velocity);
        double pidVolts = pid.calculate(velocitySignal.refresh().getValue(), velocity);
        setVoltage(feedForwardVolts + pidVolts);
    }

    @Override
    public void setVoltage(double volts) {
        motor1.setVoltage(volts);
        motor2.setVoltage(volts);
    }

    @Override
    public boolean checkNoteShot() {
        // returns whether current has risen for more than 0.1 seconds
        return currentDebouncer.calculate(currentSignal.getValue() < 15)
                && velocitySignal.getValue() * 10 * 60 > (HardwareConstants.CIM_MAX_RPM / 60)
                        * (voltageSignal.getValue() / 12);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(voltageSignal, currentSignal, velocitySignal, temperatureSignal);
        inputs.motorVoltage = voltageSignal.getValue();
        inputs.motorCurrent = currentSignal.getValue();
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorTemperature = temperatureSignal.getValue();
    }
}