package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.swerveDrive.ModuleIOInputsAutoLogged;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    // TODO: switch to tunable numbers w/ smartdash
    private static final Measure<Distance> wheelRadius = Inches.of(2); // 2"; .0508m
    // private static final Measure<Distance> trackWidth = Inches.of(22.5);

    private SwerveModuleState prevSetpoint = new SwerveModuleState(0, new Rotation2d(0));

    /**
     * Constructs a new Module (subsytem) object
     * 
     * @param io    The ModuleIO object to use
     * @param index The index of the module
     */
    public Module(ModuleIO io, int index) {
        System.out.println("[Init] Creating Module " + Integer.toString(index));
        this.io = io;
        this.index = index;

        // turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        // double prevVel = getVelocityMetersPerSec();
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    }

    /**
     * Runs the module with the specified setpoint state.
     * Must be called periodically. Returns the optimized state.
     * 
     * @param targetState The desired state of the module
     * @return The optimized state of the module
     */
    public SwerveModuleState runSetpoint(SwerveModuleState targetState, boolean isOpenLoop) {
        // Optimize state based on current angle
        var optimizedState = SwerveModuleState.optimize(targetState, getAngle());
        // optimizedState = targetState; // for testing ONLY

        io.setTurnPosition(Radians.of(optimizedState.angle.getRadians()));

        // Update velocity based on turn error
        // does some fancy things to move only in the direction you want while theres an
        // error
        // draw out the current/desired vectors, and remember that cos is like the dot
        // product,
        // it projects one vector onto the other, idk I cant make sense of it rn im
        // tired asf
        optimizedState.speedMetersPerSecond *= Math
                .cos(inputs.turnAbsolutePositionRad.in(Radians) - optimizedState.angle.getRadians());

        if (!isOpenLoop) {
            // constrian velocity based on voltage and previous velocity using motor
            // dynamics
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
        } else {
            io.setDriveVoltage(
                    Volts.of(optimizedState.speedMetersPerSecond) // NORMALIZED IN DRIVE (state speed / max linear
                                                                  // speed)
            );
        }

        prevSetpoint = optimizedState;
        return optimizedState;
    }

    /**
     * Calculates the maximum possible velocity of a DC Motor, given
     * parameters of the motor and the current velocity.
     * 
     * xₖ₊₁ = A_d xₖ + B_d uₖ where
     * A = -Kᵥ/Kₐ
     * B = 1/Kₐ
     * A_d = eᴬᵀ
     * B_d = A⁻¹(A_d − I)B
     * 
     * true max (xₖ₊₁=xₖ): x = (I − A_d)⁻¹B_duₖ
     * 
     * @param maxControlInput Maximum voltage input to the motor
     * @param currentVelocity Current velocity of the motor (previous setpoint)
     * @param dt              Time to achieve the next setpoint (period)
     * @param kV              Motor kV (Volts/RadiansPerSecond)
     * @param kA              Motor kA (Volts/RadiansPerSecondPerSecond)
     * @return The max achievable velocity given the control max control input
     */
    public static double getMaxVelocity(double maxControlInput, double currentVelocity, double dt, double kV,
            double kA) {
        double A = -kV / kA;
        double B = 1 / kA;
        double A_d = Math.exp(A * dt);
        double B_d = (1 / A) * (A_d - 1) * B;

        // System.out.println("true max (m/s): " + 1/(1 - A_d) * B_d * maxControlInput *
        // 0.0508);
        return (A_d * currentVelocity + B_d * maxControlInput);
    }

    public Translation2d getModuleTranslation() {
        return Drive.getModuleTranslations()[index];
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees.
     * Must be called periodically.
     */
    public void runCharacterization(Measure<Voltage> volts) {
        // System.out.println(volts.in(Volts));
        io.setTurnPosition(Radians.of(0.0));
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors (sets voltage to 0). */
    public void stop() {
        io.setTurnVoltage(Volts.of(0.0));
        io.setDriveVoltage(Volts.of(0.0));
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
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

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getRawPosition() {
        return new SwerveModulePosition(getRawPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive position in radians. */
    public Measure<Angle> getCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the drive velocity in radians/sec. */
    public Measure<Velocity<Angle>> getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    /** Returns the drive voltage in volts. */
    public Measure<Voltage> getCharacterizationVoltage() {
        return inputs.driveAppliedVolts;
    }

    /** Returns the drive wheel radius. */
    public static Measure<Distance> getWheelRadius() {
        return wheelRadius;
    }
}