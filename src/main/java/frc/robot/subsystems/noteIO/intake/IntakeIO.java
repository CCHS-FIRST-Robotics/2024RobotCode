package frc.robot.subsystems.noteIO.intake;

import org.littletonrobotics.junction.AutoLog;

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

    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {
    }
}