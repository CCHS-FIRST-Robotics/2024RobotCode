package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double motor1Voltage;
        public double motor1Current;
        public double motor1Velocity;

        public double motor2Voltage;
        public double motor2Current;
        public double motor2Velocity;
    }

    public default void setVoltage(double volts) {
    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }
}