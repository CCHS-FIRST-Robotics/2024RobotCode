package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    // TODO: switch to tunable numbers w/ smartdash
    private static final Measure<Distance> wheelRadius = Inches.of(2);

    private SwerveModuleState prevSetpoint = new SwerveModuleState(0, new Rotation2d(0));

    public Module(ModuleIO io, int index) {
        System.out.println("[Init] Creating Module " + Integer.toString(index));
        this.io = io;
        this.index = index;

        // turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void periodic() {
        // double prevVel = getVelocityMetersPerSec();
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    public SwerveModuleState runSetpoint(SwerveModuleState targetState) {
        // Optimize state based on current angle
        var optimizedState = SwerveModuleState.optimize(targetState, getAngle());

        io.setTurnPosition(Radians.of(optimizedState.angle.getRadians()));
        // io.setTurnVoltage(Volts.of(1));

        // Update velocity based on turn error
        // does some fancy things to move only in the direction you want while theres an error
        // draw out the current/desired vectors, and remember that cos is like the dot product,
        // it projects one vector onto the other, idk I cant make sense of it rn im tired asf
        optimizedState.speedMetersPerSecond *= Math
                .cos(inputs.turnAbsolutePositionRad.in(Radians) - optimizedState.angle.getRadians());

        
            // constrian velocity based on voltage and previous velocity using motor dynamics
            optimizedState.speedMetersPerSecond = MathUtil.clamp(
                    optimizedState.speedMetersPerSecond,
                    getMaxVelocity(-inputs.driveAverageBusVoltage.in(Volts),
                            prevSetpoint.speedMetersPerSecond / wheelRadius.in(Meters), Constants.PERIOD,
                            ModuleIO.driveKv,
                            ModuleIO.driveKa) * wheelRadius.in(Meters),
                    getMaxVelocity(inputs.driveAverageBusVoltage.in(Volts),
                            prevSetpoint.speedMetersPerSecond / wheelRadius.in(Meters), Constants.PERIOD,
                            ModuleIO.driveKv,
                            ModuleIO.driveKa) * wheelRadius.in(Meters));

            // Run drive controller
            // System.out.println(wheelRadius.in(Meters));
            double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius.in(Meters);
            io.setDriveVelocity(RadiansPerSecond.of(velocityRadPerSec));
        

        prevSetpoint = optimizedState;
        return optimizedState;
    }

    public static double getMaxVelocity(double maxControlInput, double currentVelocity, double dt, double kV,
            double kA) {
        double A = -kV / kA;
        double B = 1 / kA;
        double A_d = Math.exp(A * dt);
        double B_d = (1 / A) * (A_d - 1) * B;

        // System.out.println("true max (m/s): " + 1/(1 - A_d) * B_d * maxControlInput * 0.0508);
        return (A_d * currentVelocity + B_d * maxControlInput);
    }

    public void runCharacterization(Measure<Voltage> volts) {
        io.setTurnPosition(Radians.of(0.0));
        io.setDriveVoltage(volts);
    }

    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    // ! why is this necessary
    public void setBrakeMode() {
        io.setDriveBrakeMode(true);
        io.setTurnBrakeMode(false);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad.in(Radians)));
    }

    /** Returns the current drive position of the module in meters. */
    public double getRawPositionMeters() {
        return inputs.driveRawPositionRad.in(Radians) * wheelRadius.in(Meters);
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad.in(Radians) * wheelRadius.in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec.in(RadiansPerSecond) * wheelRadius.in(Meters);
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }
}