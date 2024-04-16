package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
    private static final double gearRatio = 100 * 48 / 22d;
    Measure<Angle> startPos = Degrees.of(-15);
    LinearFilter accelerationSetpoint = LinearFilter.backwardFiniteDifference(1, 2, Constants.PERIOD);

    SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getFalcon500Foc(1),
            gearRatio,
            1,
            .41,
            startPos.in(Radians),
            Radians.convertFrom(120, Degrees),
            true,
            Radians.convertFrom(0, Degrees));

    PIDController feedback = new PIDController(20, 0, 0);
    ArmFeedforward feedforward = new ArmFeedforward(0, .435, 12 * (3 / 319d), 0);
    TrapezoidProfile trapProf = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    RotationsPerSecond.of(5),
                    RotationsPerSecond.per(Second).of(30)));
    State prevState = new State(startPos.in(Rotations), 0);

    Measure<Voltage> appliedVoltage = Volts.of(0.0);

    public ArmIOSim() {

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(Constants.PERIOD);

        inputs.driveAppliedVolts = appliedVoltage;
        inputs.driveCurrent = Amps.of(armSim.getCurrentDrawAmps());
        inputs.absoluteArmPosition = Radians.of(armSim.getAngleRads());
        inputs.absoluteArmVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        inputs.closedLoopReference = Rotations.of(prevState.position);
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        appliedVoltage = Volts.of(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
        armSim.setInputVoltage(volts.in(Volts));
    }

    @Override
    public void setDrivePosition(Measure<Angle> position) {
        var setpoint = trapProf.calculate(
                Constants.PERIOD,
                prevState,
                new State(position.in(Rotations), 0));

        double volts = feedforward.calculate(
                setpoint.position,
                setpoint.velocity,
                accelerationSetpoint.calculate(setpoint.velocity))
                + feedback.calculate(
                        Rotations.convertFrom(armSim.getAngleRads(), Radians),
                        setpoint.position);

        setDriveVoltage(Volts.of(volts));

        prevState = setpoint;
    }
}
