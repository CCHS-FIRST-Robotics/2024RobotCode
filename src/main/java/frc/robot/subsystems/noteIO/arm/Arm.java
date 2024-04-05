package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.Supplier;
import edu.wpi.first.units.*;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private Measure<Angle> targetAngle = Degrees.of(0);
    private SysIdRoutine sysIdRoutine;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    // meters, degrees
    private static final InterpolatingDoubleTreeMap speakerAngleMap = new InterpolatingDoubleTreeMap();
    static {
        speakerAngleMap.put(1.3, 5d);
        speakerAngleMap.put(1.8, 9d);
        speakerAngleMap.put(2.3, 21d);
        speakerAngleMap.put(2.6, 23d);
        speakerAngleMap.put(2.8, 25.2d);
        speakerAngleMap.put(3d, 25.9d);
        speakerAngleMap.put(3.3, 26.1d);
        speakerAngleMap.put(3.8, 26.3d);
        speakerAngleMap.put(4.3d, 26.5d);
    }

    public Arm(ArmIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config( // Calculate around how far it's going to go (less than 90 deg)
                        Volts.per(Second).of(2),
                        Volts.of(8.75),
                        Seconds.of(5),
                        (state) -> SignalLogger.writeString("SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            io.setDriveCurrent(Amps.of(volts.in(Volts)));
                        },
                        null,
                        this));
    }

    public Command moveArm(ArmPosition position, Supplier<Pose2d> robotPose) {
        if (position == ArmPosition.SHOOT) {
            return moveToShoot(robotPose);
        }

        Measure<Angle> angle = ARM_POSITIONS.get(position);
        return runOnce(() -> setArmAngle(angle));
    }

    public Command moveToShoot(Supplier<Pose2d> robotPose) {
        return run(() -> {
            double distance = robotPose.get().getTranslation().minus(
                    SPEAKER_POSE.getTranslation()).getNorm();
            double angle = speakerAngleMap.get(distance);
            setArmAngle(Degrees.of(angle));
            Logger.recordOutput("SpeakerDistance", distance);
        });
    }

    public void setArmAngle(Measure<Angle> angle) {
        targetAngle = angle;
        io.setDrivePosition(angle);
    }

    public boolean atGoal() {
        return Math.abs(inputs.drivePosition.in(Degrees) - targetAngle.in(Degrees)) < .8;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdFull() {
        return (sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> inputs.drivePosition.in(Degrees) > 60))
                .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> inputs.drivePosition.in(Degrees) < 0))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> inputs.drivePosition.in(Degrees) > 60))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> inputs.drivePosition.in(Degrees) < 0))
                .andThen(new InstantCommand(() -> io.setDriveCurrent(Amps.of(0))));
    }
}