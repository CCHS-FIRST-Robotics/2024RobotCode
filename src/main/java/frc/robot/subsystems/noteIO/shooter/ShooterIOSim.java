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

    Measure<Velocity<Angle>> prevSetpoint = RotationsPerSecond.of(0);

    public ShooterIOSim() {

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.motor1Voltage = appliedVoltage.in(Volts);
        inputs.motor1Current = motor.getCurrentDrawAmps();
        inputs.motor1Velocity = motor.getAngularVelocityRPM() / 60;

        inputs.motor2Voltage = appliedVoltage.in(Volts);
        inputs.motor2Current = motor.getCurrentDrawAmps();
        inputs.motor2Velocity = motor.getAngularVelocityRPM() / 60;
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        appliedVoltage = volts;
        motor.setInputVoltage(volts.in(Volts));
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        double volts = feedforward.calculate(
                prevSetpoint.in(RotationsPerSecond)
                // velocity.in(RotationsPerSecond),
                // Constants.PERIOD
                )
                + feedback.calculate(
                        motor.getAngularVelocityRPM() / 60,
                        velocity.in(RotationsPerSecond));
        setVoltage(Volts.of(volts));

        prevSetpoint = velocity;
    }
    
    @Override
    public boolean upToSpeed(Measure<Velocity<Angle>> targetVelocity) {
        return motor.getAngularVelocityRPM() > targetVelocity.in(Rotations.per(Minute));
    }

}