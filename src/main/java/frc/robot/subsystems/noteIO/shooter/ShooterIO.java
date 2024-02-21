package frc.robot.subsystems.noteIO.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorVelocity;
    }

    public default void setVelocity(double velocity) {
    }

    public default void setVoltage(double volts) {
    }

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {
    }
}