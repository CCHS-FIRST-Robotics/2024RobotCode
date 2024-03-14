package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.*;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.EventCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.handoff.Handoff;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import edu.wpi.first.units.Units.*;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class EventMarkerBuilder {
    private Drive drive;
    private Intake intake;
    private Handoff handoff;
    private Shooter shooter;
    private Arm arm;

    private Command command;

    public EventMarkerBuilder(ArrayList<String> pathList, Drive drive, Intake intake, Handoff handoff,
            Shooter shooter, Arm arm) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
        this.handoff = handoff;

        // maybe change???? have it move to right in front of speaker first if not there before starting auto path?
        // command = new MoveToPose(drive, );

        command = command.andThen(new InstantCommand(
                () -> shooter.start(AutoPathConstants.SHOOT_SPEED_LEFT, AutoPathConstants.SHOOT_SPEED_RIGHT),
                shooter)
                .andThen(Commands.waitUntil(shooter::upToSpeed)
                        .andThen(
                                arm.moveArm(ArmPosition.SHOOT, drive::getPose)
                                        .alongWith(
                                                handoff.getShootCommand(
                                                        Volt.of(AutoPathConstants.HANDOFF_VOLTS),
                                                        shooter::checkNoteShot)))
                        .until(shooter::checkNoteShot)));

        for (String path : pathList) {
            addCommand(path);
        }

        command = command.andThen(
                new InstantCommand(() -> shooter.stop())
                        .alongWith(new InstantCommand(() -> handoff.stop())));

    }

    public void addCommand(String path) {
        DriveTrajectory traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);
        List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

        double driveTime = Math.max(AutoPathConstants.MAX_ARM_MOVE_TIME, choreoTrajectory.getTotalTime());
        double intakeTime = driveTime + AutoPathConstants.INTAKE_TIME;
        double shootTime = intakeTime + AutoPathConstants.MAX_ARM_MOVE_TIME;
        double totalTime = shootTime + AutoPathConstants.SHOOT_TIME;

        // start drive and more arm to inake
        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME,
                arm.moveArm(ArmPosition.INTAKE, drive::getPose)
                        .alongWith(drive.followTrajectory(traj))
                        .alongWith(intake.getIntakeCommand(Volt.of(AutoPathConstants.INTAKE_VOLTS),
                                intake::checkNoteThere))));
        // arm shoot
        eventMarkers.add(Pair.of(intakeTime,
                arm.moveArm(ArmPosition.SHOOT, drive::getPose)));

        // run handoff & shoot
        eventMarkers.add(Pair.of(shootTime,
                handoff.getShootCommand(Volt.of(AutoPathConstants.HANDOFF_VOLTS),
                        shooter::checkNoteShot)));

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
