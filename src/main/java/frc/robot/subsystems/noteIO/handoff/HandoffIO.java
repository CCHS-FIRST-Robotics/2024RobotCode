package frc.robot.subsystems.noteIO.handoff;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {
        public double motorCurrent;
        public double motorVoltage;
        public double motorPosition;
        public double motorVelocity;
        public double motorTemperature;
    }

    public default void setVoltage(Measure<Voltage> v) {
    }

    public default void updateInputs(HandoffIOInputs inputs) {
    }
}