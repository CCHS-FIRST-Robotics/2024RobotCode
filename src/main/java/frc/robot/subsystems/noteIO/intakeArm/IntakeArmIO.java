package frc.robot.subsystems.noteIO.intakeArm;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
    @AutoLog
    public static class IntakeArmIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorPosition;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(Measure<Voltage> volts) {
    }

    public default void updateInputs(IntakeArmIOInputs inputs) {
    }
}