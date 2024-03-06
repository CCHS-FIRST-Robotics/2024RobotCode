package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ARM_POSITIONS;
import static frc.robot.Constants.SPEAKER_POSE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmPosition;
import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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

    // thiefed from littleton lmao
    static {
        armAngleMap.put(3.005925, 31.0);
        armAngleMap.put(6.5, 30.0);
        armAngleMap.put(10d, 38d);
      }

    public Arm(ArmIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config( // Calculate ~ how far it's going to go (less than 90 deg)
                        Volts.per(Second).of(.8),
                        Volts.of(3),
                        Seconds.of(5),
                        (state) -> Logger.recordOutput("SysIdArmTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            io.setDriveVoltage(volts);
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
        // setArmAngle(Degrees.of(30));
    }

    public void setArmAngle(Measure<Angle> angle) {
        io.setDrivePosition(angle);
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

    private Command moveToShoot(Supplier<Pose2d> robotPose) {
        return run(() -> {
            double angle = armAngleMap.get(
                robotPose.get().getTranslation().minus(
                    SPEAKER_POSE.getTranslation()
                ).getNorm());
            setArmAngle(Degrees.of(angle));
        });
    }

    public Command moveArm(ArmPosition position, Supplier<Pose2d> robotPose) {
        if (position == ArmPosition.SHOOT) {
            return moveToShoot(robotPose);
        }

        Measure<Angle> angle = ARM_POSITIONS.get(position);
        return run(() -> setArmAngle(angle));
    }
}