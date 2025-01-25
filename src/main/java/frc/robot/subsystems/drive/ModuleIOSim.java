package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {

    public double driveKp = 0.00015; // 0.00015
    public double driveKd = 0.0;
    public double driveKi = 0.000000; // 0.000008
    public double driveKs = 0.0; // .19
    public double driveKv = 0.1362; // From NEO datasheet (473kV): 0.136194 V/(rad/s) -
                                    // https://www.wolframalpha.com/input?i=1%2F%28473+*+2pi%2F60%29+*+%2850.0+%2F+14.0%29+*+%2817.0+%2F+27.0%29+*+%2845.0+%2F+15.0%29
    public double driveKa = 0.0148;

    public double turnKp = 8;
    public double turnKd = 1.5;

    PIDController drivePID = new PIDController(driveKp, driveKi, driveKd);
    PIDController turnPID = new PIDController(turnKp, 0, turnKd);
    SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

    Measure<Velocity<Angle>> prevVelocity = RadiansPerSecond.of(0.0);

    // private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 /
    // 27.0) * (45.0 / 15.0);
    // private final double turnAfterEncoderReduction = 150.0 / 7.0;

    private DCMotorSim driveSim;
    private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004);

    // private Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() *
    // 2.0 * Math.PI);
    private Rotation2d turnAbsoluteInitPosition = new Rotation2d(0);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        System.out.println("[Init] Creating ModuleIOSim");

        turnPID.enableContinuousInput(0, 1);

        driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveKv, driveKa), DCMotor.getNEO(1), 1);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        // System.out.println("test");
        driveSim.update(Constants.PERIOD);
        turnSim.update(Constants.PERIOD);

        inputs.drivePositionRad = Radians.of(driveSim.getAngularPositionRad());
        inputs.driveVelocityRadPerSec = RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec());
        inputs.driveAppliedVolts = Volts.of(driveAppliedVolts);
        inputs.driveAverageBusVoltage = Volts.of(12);
        inputs.driveCurrentAmps = Amps.of(Math.abs(driveSim.getCurrentDrawAmps()));

        inputs.turnAbsolutePositionRad = Radians.of(
                new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition).getRadians());
        inputs.turnPositionRad = Radians.of(new Rotation2d(turnSim.getAngularPositionRad()).getRadians());
        inputs.turnVelocityRadPerSec = RadiansPerSecond.of(turnSim.getAngularVelocityRadPerSec());
        inputs.turnAppliedVolts = Volts.of(turnAppliedVolts);
        inputs.turnAverageBusVoltage = Volts.of(12);
        inputs.turnCurrentAmps = Amps.of(Math.abs(turnSim.getCurrentDrawAmps()));
    }

    public void setDriveVoltage(Measure<Voltage> volts) {
        driveAppliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(Measure<Voltage> volts) {
        turnAppliedVolts = MathUtil.clamp(volts.in(Volts), -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    public void setDriveVelocity(Measure<Velocity<Angle>> velocity) {
        // velocity = velocity.times(driveAfterEncoderReduction);

        double volts = drivePID.calculate(
                RadiansPerSecond.of(driveSim.getAngularVelocityRadPerSec()).in(Rotations.per(Minute)),
                velocity.in(Rotations.per(Minute)))
                + driveFF.calculate(prevVelocity.in(RadiansPerSecond), velocity.in(RadiansPerSecond), Constants.PERIOD);

        prevVelocity = velocity;
        setDriveVoltage(Volts.of(volts));
        // driveSim.setState(driveSim.getAngularPositionRad(),
        // velocityRadPerSec.in(RadiansPerSecond));
    }

    public void setTurnPosition(Measure<Angle> position) {
        position = Radians.of(
                MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));

        Measure<Angle> currentPosition = Radians.of(
                MathUtil.inputModulus(turnSim.getAngularPositionRad(), 0, 2 * Math.PI));

        double volts = turnPID.calculate(
                currentPosition.in(Rotations),
                position.in(Rotations));

        setTurnVoltage(Volts.of(volts));
        // turnSim.setState(turnRelativePositionRad,
        // turnSim.getAngularVelocityRadPerSec());
    }
}
