package frc.robot.subsystems.mecaDrive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public double frPositionRaw = 0.0;
    public double flPositionRaw = 0.0;
    public double brPositionRaw = 0.0;
    public double blPositionRaw = 0.0;

    public double frVelocityRaw = 0.0;
    public double flVelocityRaw = 0.0;
    public double brVelocityRaw = 0.0;
    public double blVelocityRaw = 0.0;

    public double gyroYawRad = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double frVolts, double flVolts, double brVolts, double blVolts) {
  }
}
