package frc.robot.subsystems.noteIO.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.Orchestra;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftShooterCurrent;
        public double leftShooterVoltage;
        public double leftShooterPosition;
        public double leftShooterVelocity;
        public double leftShooterTemperature;
        public double closedLoopReference;

        public double rightShooterCurrent;
        public double rightShooterVoltage;
        public double rightShooterPosition;
        public double rightShooterVelocity;
        public double rightShooterTemperature;
    }

    public default void setVelocity(Measure<Velocity<Angle>> leftVelocity, Measure<Velocity<Angle>> rightVelocity) {
    }

    public default void setVoltage(Measure<Voltage> v) {
    }

    public default boolean upToSpeed(Measure<Velocity<Angle>> leftTargetVelocity,
            Measure<Velocity<Angle>> rightTargetVelocity) {
        return false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void addToOrchestra(Orchestra orchestra, int trackNum) {
    }
}