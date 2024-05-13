package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Measure<Angle> drivePosition = Radians.of(0.0);
        public Measure<Velocity<Angle>> driveVelocity = RadiansPerSecond.of(0.0);
        public Measure<Voltage> driveAppliedVolts = Volts.of(0.0);
        public Measure<Current> driveCurrent = Amps.of(0);
        public Measure<Temperature> driveTemp = Celsius.of(0);

        public Measure<Angle> absoluteArmPosition = Radians.of(0.0);
        public Measure<Velocity<Angle>> absoluteArmVelocity = RadiansPerSecond.of(0.0);

        public Measure<Angle> rotorPosition = Radians.of(0.0);

        public Measure<Angle> closedLoopReference = Radians.of(0.0);
        public Measure<Angle> closedLoopError = Radians.of(0.0);

        public boolean faultFusedSensorOutOfSync = false;
        public boolean stickyFaultFusedSensorOutOfSync = false;
        public boolean faultRemoteSensorOutOfSync = false;
        public boolean stickyFaultRemoteSensorOutOfSync = false;
    }
    
    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(Measure<Voltage> volts) {
    }

    /** Run the drive motor at the specified voltage. */
    public default void setDriveCurrent(Measure<Current> current) {
    }

    /** Run the turn motor to the specified position. */
    public default void setDrivePosition(Measure<Angle> positionRad) {
    }

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {
    }

    public default void setCharacterizationVoltage(Measure<Voltage> volts) {
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {
    }
}