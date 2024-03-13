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

        inputs.leftMotorVoltage = appliedVoltage.in(Volts);
        inputs.leftMotorCurrent = motor.getCurrentDrawAmps();
        inputs.leftMotorVelocity = motor.getAngularVelocityRPM() / 60;

        inputs.rightMotorVoltage = appliedVoltage.in(Volts);
        inputs.rightMotorCurrent = motor.getCurrentDrawAmps();
        inputs.rightMotorVelocity = motor.getAngularVelocityRPM() / 60;
    }

    @Override
    public void setVoltage(Measure<Voltage> volts) {
        appliedVoltage = volts;
        motor.setInputVoltage(volts.in(Volts));
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
        double volts = feedforward.calculate(
                prevSetpoint.in(RotationsPerSecond)
        // velocity.in(RotationsPerSecond),
        // Constants.PERIOD
        )
                + feedback.calculate(
                        motor.getAngularVelocityRPM() / 60,
                        leftVelocity.in(RotationsPerSecond));
        setVoltage(Volts.of(volts));

        prevSetpoint = leftVelocity;
    }

    @Override
    public boolean upToSpeed(Measure<Velocity<Angle>> leftTargetVelocity,
            Measure<Velocity<Angle>> rightTargetVelocity) {
        return motor.getAngularVelocityRPM() > leftTargetVelocity.in(Rotations.per(Minute));
    }
}