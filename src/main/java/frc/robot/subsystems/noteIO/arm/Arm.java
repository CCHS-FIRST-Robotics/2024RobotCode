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
import frc.robot.HardwareConstants;

// rev sucks
public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private SysIdRoutine sysIdRoutine;

    private static final InterpolatingDoubleTreeMap speakerAngleMap = new InterpolatingDoubleTreeMap();

    private Measure<Angle> targetAngle = Degrees.of(0);

    // meters, degrees
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

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    // ! looks optimizable
    public void setArmAngle(Measure<Angle> angle) {
        io.setDrivePosition(angle);
        targetAngle = angle;
    }

    public boolean atGoal() {
        return Math.abs(getArmAngle().in(Degrees) - targetAngle.in(Degrees)) < .8;
    }

    public Measure<Angle> getArmAngle() {
        return inputs.drivePosition;
    }

    public Measure<Velocity<Angle>> getArmVelocity() {
        return inputs.driveVelocity;
    }

    public Translation2d getEndEffectorPosition() {
        return new Translation2d(HardwareConstants.ARM_LENGTH.in(Meters), new Rotation2d(getArmAngle().in(Radians)))
                .plus(HardwareConstants.ARM_OFFSET);
    }

    public Command alignWithTarget(Supplier<Translation2d> translationToTargetGround, Supplier<Pose3d> targetPose) {
        return run(() -> {
            Translation2d tranlationToTargetHigh = new Translation2d(translationToTargetGround.get().getNorm(),
                    targetPose.get().getZ());
            Rotation2d targetArmAngle = tranlationToTargetHigh.minus(HardwareConstants.ARM_OFFSET).getAngle();
            setArmAngle(Radians.of(Math.PI / 2.0 - targetArmAngle.getRadians())); // add 90 degrees since 0 is vertical
        });
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

    public Command moveArm(ArmPosition position, Supplier<Pose2d> robotPose) {
        if (position == ArmPosition.SHOOT) {
            return moveToShoot(robotPose);
            // return runOnce(() ->
            // setArmAngle(Constants.ARM_POSITIONS.get(ArmPosition.SPEAKER)));
        }

        Measure<Angle> angle = ARM_POSITIONS.get(position);
        return runOnce(() -> setArmAngle(angle));
    }

    // dont want to remove this but we probably shouldnt be using it
    // public Supplier<Pose2d> getPosFromPath(String path, double eventTime) {
    // ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);

    // double timeToEnd = choreoTrajectory.getTotalTime();

    // for (int i = 0; i < (int) (timeToEnd / Constants.PERIOD) + 2; i++) {
    // double time = i * Constants.PERIOD;
    // ChoreoTrajectoryState state = choreoTrajectory.sample(time);

    // if (time >= eventTime) {
    // Translation2d translationToTargetGround = new Translation2d(state.x,
    // state.y);
    // Pose3d targetPose = new Pose3d(new Pose2d(state.x, state.y, new
    // Rotation2d(state.heading)));

    // Translation2d armOffset = getArmOffset();
    // Translation2d tranlationToTargetHigh = new
    // Translation2d(translationToTargetGround.getNorm(),
    // targetPose.getZ());
    // Rotation2d targetArmAngle =
    // tranlationToTargetHigh.minus(armOffset).getAngle();

    // return () -> new Pose2d(translationToTargetGround, targetArmAngle);
    // }
    // }

    // return null;
    // }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdFull() {
        return (sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> getArmAngle().in(Degrees) > 60))
                .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> getArmAngle().in(Degrees) < 0))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> getArmAngle().in(Degrees) > 60))
                .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> getArmAngle().in(Degrees) < 0))
                .andThen(new InstantCommand(() -> io.setDriveCurrent(Amps.of(0))));
    }
}