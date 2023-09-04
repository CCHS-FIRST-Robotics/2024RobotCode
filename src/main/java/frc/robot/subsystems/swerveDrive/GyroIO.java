package frc.robot.subsystems.swerveDrive;

import org.littletonrobotics.junction.AutoLog;

// TODO: do we need any raw acc data? I don't think we have any use for it
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    // "raw" (integrated from velocity)
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;

    // "fused" (combined with magnetometer or accelerometer)
    public double rollFusedPositionRad = 0.0;
    public double pitchFusedPositionRad = 0.0;
    public double yawFusedPositionRad = 0.0;

    // raw velocities
    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}