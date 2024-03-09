package frc.robot.subsystems.noteIO.intakeBase;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeBaseIO {
    @AutoLog
    public static class IntakeBaseIOInputs {
        public double motor1Voltage;
        public double motor1Current;
        public double motor1Velocity;
        public double motor1Temperature;

        public double motor2Voltage;
        public double motor2Current;
        public double motor2Velocity;
        public double motor2Temperature;
    }

    public default void setVoltage(double volts) {
    }

    public default void updateInputs(IntakeBaseIOInputsAutoLogged inputs) {
    }
}