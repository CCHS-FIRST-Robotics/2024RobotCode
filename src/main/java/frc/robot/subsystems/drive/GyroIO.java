package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        // "raw" (integrated from velocity)
        public Measure<Angle> rollPosition = Radians.of(0.0);
        public Measure<Angle> pitchPosition = Radians.of(0.0);
        public Measure<Angle> yawPosition = Radians.of(0.0);

        // raw velocities
        public Measure<Velocity<Angle>> rollVelocity = RadiansPerSecond.of(0.0);
        public Measure<Velocity<Angle>> pitchVelocity = RadiansPerSecond.of(0.0);
        public Measure<Velocity<Angle>> yawVelocity = RadiansPerSecond.of(0.0);
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }
}