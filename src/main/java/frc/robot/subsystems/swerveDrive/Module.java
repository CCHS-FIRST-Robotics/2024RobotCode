package frc.robot.subsystems.swerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.SwerveKinematicUtils;

import org.littletonrobotics.junction.Logger;

public class Module {
    
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    
    // TODO: switch to tunable numbers w/ smartdash
    private static final Measure<Distance> wheelRadius = Inches.of(2); // 2"; .0508m
    private static final Measure<Distance> trackWidth = Inches.of(22.5);

    private static final Measure<Velocity<Velocity<Distance>>> maxCollinearAcceleration = MetersPerSecondPerSecond.of(8);
    private static final Measure<Velocity<Velocity<Distance>>> maxOrthogonalAcceleration = MetersPerSecondPerSecond.of(8);

    private SwerveModuleState prevSetpoint = new SwerveModuleState(0, new Rotation2d(0));

    double kV = 0.1362;
    double kA = 0.0148;

    /**
     * Constructs a new Module (subsytem) object
     * 
     * @param io The ModuleIO object to use
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
        // double acceleration = (getVelocityMetersPerSec() - prevVel) / 0.02;
        // if (Math.abs(acceleration) > 50) System.out.println(acceleration);

        // until we figure out how to use the logger
        // SmartDashboard.putNumber("Drive Position", Units.radiansToDegrees(inputs.drivePositionRad));
        // SmartDashboard.putNumber("Drive Velocity", Units.radiansToDegrees(inputs.driveVelocityRadPerSec));
        // SmartDashboard.putNumber("Turn Absolute Position", Units.radiansToDegrees(inputs.turnAbsolutePositionRad));
        // SmartDashboard.putNumber("Turn Relative Position", Units.radiansToDegrees(inputs.turnPositionRad));
        // SmartDashboard.putNumber("Turn Velocity", Units.radiansToDegrees(inputs.turnVelocityRadPerSec));
    }

    /**
     * Runs the module with the specified setpoint state. Must be called periodically. Returns the
     * optimized state. Additionally to runSetpoint(targetState) this method constraints 
     * the module accelerations
     * 
     * @param targetState The desired state of the module
     * @return The optimized state of the module
     */
    public SwerveModuleState runSetpoint(SwerveModuleState targetState, Twist2d chassisAcceleration, double chassisAngularVelocity) {
        // Optimize state based on current angle
        var optimizedState = SwerveModuleState.optimize(targetState, getAngle());

        // Run turn controller
        // io.setTurnVoltage(
        //     turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians())
        // );
        io.setTurnPosition(Radians.of(optimizedState.angle.getRadians()));

        // Update velocity based on turn error
        // does some fancy things to move only in the direction you want while theres an error
        // draw out the current/desired vectors, and remember that cos is like the dot product, 
        // it projects one vector onto the other, idk I cant make sense of it rn im tired asf
        optimizedState.speedMetersPerSecond *= Math.cos(inputs.turnAbsolutePositionRad.in(Radians) - optimizedState.angle.getRadians());
        optimizedState.speedMetersPerSecond = MathUtil.clamp(
            optimizedState.speedMetersPerSecond,
            getMaxVelocity(-12, prevSetpoint.speedMetersPerSecond / wheelRadius.in(Meters), Constants.PERIOD, kV, kA) * wheelRadius.in(Meters),
            getMaxVelocity(12, prevSetpoint.speedMetersPerSecond / wheelRadius.in(Meters), Constants.PERIOD, kV, kA) * wheelRadius.in(Meters)
        );

        Translation2d moduleAcceleration = SwerveKinematicUtils.getModuleAccelerations(
            getModuleTranslation(), chassisAcceleration, chassisAngularVelocity
        ); // amx, amy - field relative
        moduleAcceleration = new Translation2d(moduleAcceleration.getNorm(), optimizedState.angle); // convert to module relative (i.e., x is in the direction of the module)
        if (moduleAcceleration.getX() > maxCollinearAcceleration.in(MetersPerSecondPerSecond)) {
            System.out.print("Collinear slippage detected");
        }
        if (moduleAcceleration.getY() > maxOrthogonalAcceleration.in(MetersPerSecondPerSecond)) {
            System.out.print("Orthogonal slippage detected");
        }

        // Run drive controller
        Measure<Velocity<Angle>> velocityRadPerSec = RadiansPerSecond.of(optimizedState.speedMetersPerSecond / wheelRadius.in(Meters));
        io.setDriveVelocity(velocityRadPerSec);

        prevSetpoint = optimizedState; 
        return optimizedState;
    }

    /**
     * Runs the module with the specified setpoint state. Must be called periodically. Returns the
     * optimized state.
     * 
     * @param targetState The desired state of the module
     * @return The optimized state of the module
     */
    public SwerveModuleState runSetpoint(SwerveModuleState targetState) {
        // Optimize state based on current angle
        var optimizedState = SwerveModuleState.optimize(targetState, getAngle());

        // Run turn controller
        // io.setTurnVoltage(
        //     turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians())
        // );
        io.setTurnPosition(Radians.of(optimizedState.angle.getRadians()));

        // Update velocity based on turn error
        // does some fancy things to move only in the direction you want while theres an error
        // draw out the current/desired vectors, and remember that cos is like the dot product, 
        // it projects one vector onto the other, idk I cant make sense of it rn im tired asf
        optimizedState.speedMetersPerSecond *= Math.cos(inputs.turnAbsolutePositionRad.in(Radians) - optimizedState.angle.getRadians());
        optimizedState.speedMetersPerSecond = MathUtil.clamp(
            optimizedState.speedMetersPerSecond,
            getMaxVelocity(-12, prevSetpoint.speedMetersPerSecond / wheelRadius.in(Meters), Constants.PERIOD, kV, kA) * wheelRadius.in(Meters),
            getMaxVelocity(12, prevSetpoint.speedMetersPerSecond / wheelRadius.in(Meters), Constants.PERIOD, kV, kA) * wheelRadius.in(Meters)
        );

        // Run drive controller
        Measure<Velocity<Angle>> velocityRadPerSec = RadiansPerSecond.of(optimizedState.speedMetersPerSecond / wheelRadius.in(Meters));
        io.setDriveVelocity(velocityRadPerSec);

        prevSetpoint = optimizedState; 
        return optimizedState;
    }
    
    /**
     * Calculates the maximum possible velocity of a DC Motor, given
     * parameters of the motor and the current velocity.
     * 
     * xₖ₊₁ = A_d xₖ + B_d uₖ where
     *   A = -Kᵥ/Kₐ
     *   B = 1/Kₐ
     *   A_d = eᴬᵀ
     *   B_d = A⁻¹(A_d − I)B
     * 
     * true max (xₖ₊₁=xₖ): x = (I − A_d)⁻¹B_duₖ
     * 
     * @param maxControlInput Maximum voltage input to the motor
     * @param currentVelocity Current velocity of the motor (previous setpoint)
     * @param dt Time to achieve the next setpoint (period)
     * @param kV Motor kV (Volts/RadiansPerSecond)
     * @param kA Motor kA (Volts/RadiansPerSecondPerSecond)
     * @return The max achievable velocity given the control max control input
     */
    public static double getMaxVelocity(double maxControlInput, double currentVelocity, double dt, double kV, double kA) {
        double A = -kV / kA;
        double B = 1 / kA;
        double A_d = Math.exp(A * dt);
        double B_d = (1 / A) * (A_d - 1) * B;

        // System.out.println("true max (m/s): " + 1/(1 - A_d) * B_d * maxControlInput * 0.0508);
        return (A_d * currentVelocity + B_d * maxControlInput);
    }

    public Translation2d getModuleTranslation() {
        return Drive.getModuleTranslations()[index];
    }

    /**
     * Runs the module with the specified voltage while controlling to zero degrees. Must be called
     * periodically.
     */
    public void runCharacterization(Measure<Voltage> volts) {
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
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad.in(Radians)));
    }

    /** Returns the current drive position of the module in meters. */
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

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the drive velocity in radians/sec. */
    public Measure<Velocity<Angle>> getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    /** Returns the drive wheel radius. */
    public static Measure<Distance> getWheelRadius() {
        return wheelRadius;
    }
}
