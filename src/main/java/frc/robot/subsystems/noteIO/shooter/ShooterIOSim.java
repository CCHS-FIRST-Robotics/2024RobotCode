package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.noteIO.intakeArm.IntakeArmIO.IntakeArmIOInputs;

public class ShooterIOSim implements ShooterIO {
    Measure<Voltage> appliedVoltage = Volts.of(0);
    DCMotorSim motor = new DCMotorSim(DCMotor.getFalcon500(1), 1, .0025);

    PIDController feedback = new PIDController(0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 12 * (3 / 319d), 0);

    double prevSetpoint;

    public ShooterIOSim() {

    }

    public void updateInputs(IntakeArmIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.motorVoltage = appliedVoltage.in(Volts);
        inputs.motorCurrent = motor.getCurrentDrawAmps();
        inputs.motorVelocity = motor.getAngularVelocityRPM() * 60;
    }

    public void setVoltage(double volts) {
        appliedVoltage = Volts.of(volts);
        motor.setInputVoltage(volts);
    }

    public void setVelocity(double velocity) {
        double volts = feedforward.calculate(
                prevSetpoint,
                velocity,
                Constants.PERIOD)
                + feedback.calculate(
                        motor.getAngularVelocityRPM() * 60,
                        velocity);
        setVoltage(volts);

        prevSetpoint = velocity;
    }

    public boolean upToSpeed() {
        return motor.getAngularVelocityRPM() > 5000;
    }

}