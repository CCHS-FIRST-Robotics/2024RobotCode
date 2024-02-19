package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

// interface that sets up variables to be logged
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(double volts) {
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }
}