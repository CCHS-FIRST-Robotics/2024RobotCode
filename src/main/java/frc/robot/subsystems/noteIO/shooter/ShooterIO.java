package frc.robot.subsystems.noteIO.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftMotorCurrent;
        public double leftMotorVoltage;
        public double leftMotorPosition;
        public double leftMotorVelocity;
        public double leftMotorTemperature;

        public double rightMotorCurrent;
        public double rightMotorVoltage;
        public double rightMotorPosition;
        public double rightMotorVelocity;
        public double rightMotorTemperature;

        public double closedLoopReference;
    }

    public default void setVelocity(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
    }

    public default void setVoltage(Measure<Voltage> v) {
    }

    public default boolean upToSpeed(Measure<Velocity<Angle>> leftTargetVelocity,
            Measure<Velocity<Angle>> rightTargetVelocity) {
        return false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }
}