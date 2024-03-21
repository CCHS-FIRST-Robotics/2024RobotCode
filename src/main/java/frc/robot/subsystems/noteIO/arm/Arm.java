package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ARM_POSITIONS;
import static frc.robot.Constants.SPEAKER_POSE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import edu.wpi.first.units.*;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
// import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// import com.choreo.lib.Choreo;
// import com.choreo.lib.ChoreoTrajectory;
// import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.Orchestra;

// rev sucks
public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    @SuppressWarnings({ "unused" })
    private SysIdRoutine sysIdRoutine;

    // length and position of the arm in relation to the robot's center
    private final Measure<Distance> armLength = Inches.of(16); // TODO: set this
    private final Translation2d armOffset = new Translation2d(0.0, .425); // TODO: set this

    /** Arm angle look up table key: meters, values: degrees */
    private static final InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();

    @AutoLogOutput
    private Measure<Angle> targetAngle = Degrees.of(0);

    // meters, degrees
    static {
        armAngleMap.put(1.3, 5d);
        armAngleMap.put(1.8, 9d);
        armAngleMap.put(2.3, 15d);
        armAngleMap.put(2.8, 20d);
        armAngleMap.put(3.3, 24d);
        armAngleMap.put(3.8, 26d);
    }

    public Arm(ArmIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config( // Calculate ~ how far it's going to go (less than 90 deg)
                        Volts.per(Second).of(.2),
                        Volts.of(.5),
                        Seconds.of(2),
                        (state) -> Logger.recordOutput("SysIdArmTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            io.setDriveCurrent(Amps.of(6 * Math.cos(getArmAngle().in(Radians)) + volts.in(Volts))); // literal slander
                        },
                        null,
                        this));
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        // System.out.println("testingggg");

        // trust!
        // io.setDriveVoltage(Volts.of(1));
        // setArmAngle(Degrees.of(10));
        // io.setDriveCurrent(Amps.of(8));
    }

    public void setArmAngle(Measure<Angle> angle) {
        // System.out.println("arm ahh");
        io.setDrivePosition(angle);
        targetAngle = angle;
    }

    /**
     * @return true if the arm is within .1 degrees of the goal (not setpoint)
     */
    @AutoLogOutput
    public boolean isAtGoal() {
        return Math.abs(getArmAngle().in(Degrees) - targetAngle.in(Degrees)) < 1;
    }

    @AutoLogOutput
    public boolean isUnderStage() {
        return getArmAngle().in(Degrees) < -22;
    }

    public Measure<Angle> getArmAngle() {
        return inputs.drivePosition;
    }

    public Measure<Velocity<Angle>> getArmVelocity() {
        return inputs.driveVelocity;
    }

    public Translation2d getArmOffset() {
        return armOffset;
    }

    public Translation2d getEndEffectorPosition() {
        return new Translation2d(armLength.in(Meters), new Rotation2d(getArmAngle().in(Radians))).plus(armOffset);
    }

    public Command alignWithTarget(Supplier<Translation2d> translationToTargetGround, Supplier<Pose3d> targetPose) {
        return run(() -> {
            Translation2d armOffset = getArmOffset();
            Translation2d tranlationToTargetHigh = new Translation2d(translationToTargetGround.get().getNorm(),
                    targetPose.get().getZ());
            Rotation2d targetArmAngle = tranlationToTargetHigh.minus(armOffset).getAngle();
            setArmAngle(Radians.of(Math.PI / 2.0 - targetArmAngle.getRadians())); // add 90 degrees since 0 is vertical
        });
    }

    public Command moveToShoot(Supplier<Pose2d> robotPose) {
        return run(() -> {
            double distance = robotPose.get().getTranslation().minus(
                    SPEAKER_POSE.getTranslation()).getNorm();
            double angle = armAngleMap.get(distance);
            setArmAngle(Degrees.of(angle));
            Logger.recordOutput("SpeakerDistance", distance);
        });
    }

    public Command moveArm(ArmPosition position, Supplier<Pose2d> robotPose) {
        if (position == ArmPosition.SHOOT) {
            // return moveToShoot(robotPose);
            return runOnce(() -> setArmAngle(Constants.ARM_POSITIONS.get(ArmPosition.SPEAKER)));
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

    public void addToOrchestra(Orchestra orchestra, int trackNum) {
        io.addToOrchestra(orchestra, trackNum);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdFull() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}