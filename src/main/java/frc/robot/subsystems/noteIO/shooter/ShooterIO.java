package frc.robot.subsystems.noteIO.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVelocity(double velocity) {
    }

    public default void setVoltage(double volts) {
    }

    public default boolean upToSpeed() {
        return false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }
}