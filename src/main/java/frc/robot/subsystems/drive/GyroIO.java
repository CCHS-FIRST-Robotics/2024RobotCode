package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

// TODO: do we need any raw acc data? I don't think we have any use for it
// TODO: dual imu??? https://arxiv.org/pdf/2107.02632.pdf
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        // "raw" (integrated from velocity)
        public Measure<Angle> rollPosition = Radians.of(0.0);
        public Measure<Angle> pitchPosition = Radians.of(0.0);
        public Measure<Angle> yawPosition = Radians.of(0.0);

        // "fused" (combined with magnetometer or accelerometer)
        public Measure<Angle> rollFusedPosition = Radians.of(0.0);
        public Measure<Angle> pitchFusedPosition = Radians.of(0.0);
        public Measure<Angle> yawFusedPosition = Radians.of(0.0);

        // raw velocities
        public Measure<Velocity<Angle>> rollVelocity = RadiansPerSecond.of(0.0);
        public Measure<Velocity<Angle>> pitchVelocity = RadiansPerSecond.of(0.0);
        public Measure<Velocity<Angle>> yawVelocity = RadiansPerSecond.of(0.0);
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }
}