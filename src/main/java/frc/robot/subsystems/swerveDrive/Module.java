
package frc.robot.subsystems.swerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

public class Module {
    
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    
    // TODO: switch to tunable numbers w/ smartdash
    private static final double wheelRadius = 0.0508; // 2"

    public Module(ModuleIO io, int index) {
        System.out.println("[Init] Creating Module " + Integer.toString(index));
        this.io = io;
        this.index = index;
    
        // turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        double prevVel = getVelocityMetersPerSec();
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module" + Integer.toString(index), inputs);
        double acceleration = (getVelocityMetersPerSec() - prevVel) / 0.02;
        // if (Math.abs(acceleration) > 50) System.out.println(acceleration);

        // until we figure out how to use the logger
        SmartDashboard.putNumber("Drive Position", Units.radiansToDegrees(inputs.drivePositionRad));
        SmartDashboard.putNumber("Drive Velocity", Units.radiansToDegrees(inputs.driveVelocityRadPerSec));
        SmartDashboard.putNumber("Turn Absolute Position", Units.radiansToDegrees(inputs.turnAbsolutePositionRad));
        SmartDashboard.putNumber("Turn Relative Position", Units.radiansToDegrees(inputs.turnPositionRad));
        SmartDashboard.putNumber("Turn Velocity", Units.radiansToDegrees(inputs.turnVelocityRadPerSec));
    }

    /**
    * Runs the module with the specified setpoint state. Must be called periodically. Returns the
    * optimized state.
    */
    public SwerveModuleState runSetpoint(SwerveModuleState targetState) {
        // Optimize state based on current angle
        var optimizedState = SwerveModuleState.optimize(targetState, getAngle());

        // Run turn controller
        // io.setTurnVoltage(
        //     turnFeedback.calculate(getAngle().getRadians(), optimizedState.angle.getRadians())
        // );
        io.setTurnPosition(optimizedState.angle.getRadians());

        // Update velocity based on turn error
        // does some fancy things to move only in the direction you want while theres an error
        // draw out the current/desired vectors, and remember that cos is like the dot product, 
        // it projects one vector onto the other, idk I cant make sense of it rn im tired asf
        optimizedState.speedMetersPerSecond *= Math.cos(inputs.turnAbsolutePositionRad - optimizedState.angle.getRadians());

        // Run drive controller
        double velocityRadPerSec = optimizedState.speedMetersPerSecond / wheelRadius;
        io.setDriveVelocity(velocityRadPerSec);

        return optimizedState;
    }

    /**
   * Runs the module with the specified voltage while controlling to zero degrees. Must be called
   * periodically.
   */
    public void runCharacterization(double volts) {
        // io.setTurnVoltage(turnFeedback.calculate(getAngle().getRadians(), 0.0));
        // io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * wheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * wheelRadius;
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
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    /** Returns the drive wheel radius. */
    public static double getWheelRadius() {
        return wheelRadius;
    }
}
