package frc.robot.subsystems.noteIO.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

public interface ArmIO {
    
    @AutoLog
    public static class ArmIOInputs {
        public Measure<Angle> drivePosition = Radians.of(0.0);
        public Measure<Velocity<Angle>> driveVelocity = RadiansPerSecond.of(0.0);
        public Measure<Voltage> driveAppliedVolts = Volts.of(0.0);
        public Measure<Current> driveCurrent = Amps.of(0);
        public Measure<Temperature> driveTemp = Celsius.of(0);
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(Measure<Voltage> volts) {}

    /** Run the turn motor to the specified position. */
    public default void setDrivePosition(Measure<Angle> positionRad) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}

    public default void setMusicTrack(String path) {}

    public default void playMusic() {}

    public default void pauseMusic() {}

    public default void stopMusic() {}

}