package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.*;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.EventCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.intakeArm.IntakeArm;
import frc.robot.subsystems.noteIO.intakeGround.IntakeGround;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import edu.wpi.first.units.Units.*;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class EventMarkerBuilder {

    private List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

    private ArrayList<String> pathList;
    private Drive drive;
    private IntakeGround intake;
    private IntakeArm handoff;
    private Shooter shooter;
    private Arm arm;

    private String path;
    private double totalTime;

    private Command command;

    public EventMarkerBuilder(ArrayList<String> pathList, Drive drive, IntakeArm handoff,
            Shooter shooter, Arm arm) {
        this.drive = drive;
        // this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
        this.handoff = handoff;

        command = new InstantCommand(() -> shooter.start(RotationsPerSecond.of(95)), shooter)
                    .andThen(Commands.waitUntil(shooter::upToSpeed)
                    .andThen(
                        arm.moveArm(ArmPosition.SHOOT, drive::getPose)
                        .alongWith(handoff.getShootCommand(Volt.of(AutoPathConstants.INTAKE_HANDOFF_VOLTS), shooter::checkNoteShot))
                    ).until(shooter::checkNoteShot)
        );

        for (String path : pathList) {
            addCommand(path);
        }

        command = command.andThen(
            new InstantCommand(() -> shooter.stop())
            .alongWith(new InstantCommand(() -> handoff.stop()))
        );

    }

    public void addCommand(String path) {
        this.path = path;
        DriveTrajectory traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);
        List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

        double driveTime = Math.max(AutoPathConstants.Q_MAX_ARM_MOVE_TIME, choreoTrajectory.getTotalTime());
        double intakeTime = driveTime + AutoPathConstants.Q_INTAKE_TIME;
        double shootTime = intakeTime + AutoPathConstants.Q_MAX_ARM_MOVE_TIME;
        double totalTime = shootTime + AutoPathConstants.Q_SHOOT_TIME; // + time to shoot

        /*
         * quokka robot
         */
        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME,
                arm.moveArm(ArmPosition.INTAKE, drive::getPose)
                .alongWith(drive.followTrajectory(traj))
                .alongWith(handoff.getHandoffCommand(Volt.of(AutoPathConstants.INTAKE_HANDOFF_VOLTS)))
                // .alongWith(new InstantCommand(() -> System.out.println("sdhfdhfdsbdfhy")))
        ));
        // arm shoot
        eventMarkers.add(Pair.of(intakeTime,
                arm.moveArm(ArmPosition.SHOOT, drive::getPose))); // use current pose because it might not be following the path well
        // run handoff & shoot
        eventMarkers.add(Pair.of(shootTime,
                handoff.getShootCommand(Volt.of(AutoPathConstants.INTAKE_VOLTS), shooter::checkNoteShot)));

        /*
         * final robot
         */
        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME,
                arm.moveArm(ArmPosition.INTAKE, drive::getPose)
                .alongWith(drive.followTrajectory(traj))
                // along with intake
        ));
        // when intaked handoff
        // when handoff arm shoot
        eventMarkers.add(Pair.of(intakeTime,
                arm.moveArm(ArmPosition.SHOOT, drive::getPose)));
        // shoot

        if (command == null) {
            command = (new AutoCommand(eventMarkers, totalTime));
        } else {
            command = command.andThen((new AutoCommand(eventMarkers, totalTime)));
        }
    }

    public Command getCommandSequence() {
        return command;
    }
}
