package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

public interface ModuleIO {
    public double driveKv = 0.136898; // ! tf
    public double driveKa = 0.020864;

    @AutoLog
    public static class ModuleIOInputs {
        public Measure<Angle> driveRawPositionRad = Radians.of(0.0);
        public Measure<Angle> drivePositionRad = Radians.of(0.0);
        public Measure<Velocity<Angle>> driveVelocityRadPerSec = RadiansPerSecond.of(0.0);
        public Measure<Voltage> driveAppliedVolts = Volts.of(0.0);
        public Measure<Voltage> driveAverageBusVoltage = Volts.of(12);
        public Measure<Current> driveCurrentAmps = Amps.of(0);
        public Measure<Temperature> driveTempCelcius = Celsius.of(0);

        public Measure<Angle> turnAbsolutePositionRad = Radians.of(0.0);
        public Measure<Angle> turnPositionRad = Radians.of(0.0);
        public Measure<Velocity<Angle>> turnVelocityRadPerSec = RadiansPerSecond.of(0.0);
        public Measure<Voltage> turnAppliedVolts = Volts.of(0.0);
        public Measure<Voltage> turnAverageBusVoltage = Volts.of(12);
        public Measure<Current> turnCurrentAmps = Amps.of(0.0);
        public Measure<Temperature> turnTempCelcius = Celsius.of(0.0);
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(Measure<Voltage> volts) {}

    public default void setTurnVoltage(Measure<Voltage> volts) {}

    public default void setDriveVelocity(Measure<Velocity<Angle>> velocityRadPerSec) {}

    public default void setTurnPosition(Measure<Angle> positionRad) {}

    public default void setDriveBrakeMode(boolean enable) {}

    public default void setTurnBrakeMode(boolean enable) {}
}