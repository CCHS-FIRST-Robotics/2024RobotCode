
package frc.robot.subsystems.swerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Module {
    
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    
    // TODO: switch to tunable numbers w/ smartdash
    private static final Measure<Distance> wheelRadius = Units.Inches.of(2); // 2"; .0508m

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

    /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
    public void updateInputs() {
        io.updateInputs(inputs);
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        // double prevVel = getVelocityMetersPerSec();
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

        // Run drive controller
        Measure<Velocity<Angle>> velocityRadPerSec = RadiansPerSecond.of(optimizedState.speedMetersPerSecond / wheelRadius.in(Meters));
        io.setDriveVelocity(velocityRadPerSec);

        return optimizedState;
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
