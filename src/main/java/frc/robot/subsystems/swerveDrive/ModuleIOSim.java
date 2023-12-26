package frc.robot.subsystems.swerveDrive;


import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import org.ejml.simple.SimpleMatrix;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {

    private static final double driveKp = 0.00015; // 0.00015
    private static final double driveKd = 0.0;
    private static final double driveKi = 0.000000; // 0.000008
    private static final double driveKs = 0.0; //.19
    private static final double driveKv = 0.1362; // From NEO datasheet (473kV): 0.136194 V/(rad/s) - https://www.wolframalpha.com/input?i=1%2F%28473+*+2pi%2F60%29+*+%2850.0+%2F+14.0%29+*+%2817.0+%2F+27.0%29+*+%2845.0+%2F+15.0%29
    private static final double driveKa = 0.0148;

    private static final double turnKp = 8;
    private static final double turnKd = 1.5;

    PIDController drivePID = new PIDController(driveKp, driveKi, driveKd);
    PIDController turnPID = new PIDController(turnKp, 0, turnKd);
    SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

    Measure<Velocity<Angle>> prevVelocity = RadiansPerSecond.of(0.0);

    private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private DCMotorSim driveSim;
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004);

    // private Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private Rotation2d turnAbsoluteInitPosition = new Rotation2d(0);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        System.out.println("[Init] Creating ModuleIOSim");

        turnPID.enableContinuousInput(0, 1);

        driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveKv, driveKa), DCMotor.getNEO(1), driveAfterEncoderReduction);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        // System.out.println("test");
        driveSim.update(Constants.PERIOD);
        turnSim.update(Constants.PERIOD);

        inputs.drivePositionRad = Radians.of(driveSim.getAngularPositionRad());
        inputs.driveVelocityRadPerSec = RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec());
        inputs.driveAppliedVolts = Volts.of(driveAppliedVolts);
        inputs.driveCurrentAmps = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

        inputs.turnAbsolutePositionRad =
            Radians.of(
                new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition).getRadians()
            );
        inputs.turnPositionRad = Radians.of(new Rotation2d(turnSim.getAngularPositionRad()).getRadians());
        inputs.turnVelocityRadPerSec = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnAppliedVolts = Volts.of(turnAppliedVolts);
        inputs.turnCurrentAmps = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));
    }

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    public void setDriveVelocity(Measure<Velocity<Angle>> velocity) {
        // velocity = velocity.times(driveAfterEncoderReduction);

        double volts = drivePID.calculate(
            RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec()).in(Rotations.per(Minute)),
            velocity.in(Rotations.per(Minute))
        ) + driveFF.calculate(prevVelocity.in(RadiansPerSecond), velocity.in(RadiansPerSecond), Constants.PERIOD);

        prevVelocity = velocity;
        setDriveVoltage(volts);
        // driveSim.setState(driveSim.getAngularPositionRad(), velocityRadPerSec.in(RadiansPerSecond));
    }

    public void setTurnPosition(Measure<Angle> position) {
        position = Radians.of(
            MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI)
        );

        Measure<Angle> currentPosition = Radians.of(
            MathUtil.inputModulus(turnSim.getAngularPositionRad(), 0, 2 * Math.PI)
        );

        double volts = turnPID.calculate(
            currentPosition.in(Rotations),
            position.in(Rotations)
        );

        setTurnVoltage(volts);
        // turnSim.setState(turnRelativePositionRad, turnSim.getAngularVelocityRadPerSec());
    }
}

