package frc.robot.subsystems.noteIO.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double motor1Current;
        public double motor1Voltage;
        public double motor1Position;
        public double motor1Velocity;
        public double motor1Temperature;

        public double motor2Current;
        public double motor2Voltage;
        public double motor2Position;
        public double motor2Velocity;
        public double motor2Temperature;
    }

    public default void setVelocity(double velocity) {
    }

    public default void setVoltage(double volts) {
    }

    public default boolean upToSpeed(double targetVelocity) {
        return false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }
}