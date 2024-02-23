package frc.robot.subsystems.noteIO.intakeArm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
    @AutoLog
    public static class IntakeArmIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(double volts) {
    }

    public default void updateInputs(IntakeArmIOInputsAutoLogged inputs) {
    }
}