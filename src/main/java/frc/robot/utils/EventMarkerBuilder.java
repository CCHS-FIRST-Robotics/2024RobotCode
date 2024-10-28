package frc.robot.utils;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.Pair;
import com.choreo.lib.*;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.handoff.Handoff;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import frc.robot.Constants.*;


public final class EventMarkerBuilder {
    private Drive drive;
    private Intake intake;
    private Handoff handoff;
    private Shooter shooter;
    private Arm arm;

    private Command command;

    ArrayList<Double> timestamps;

    public EventMarkerBuilder(
        ArrayList<String> pathList, 
        Drive drive, 
        Arm arm,
        Intake intake, 
        Handoff handoff,
        Shooter shooter
    ){
        this.drive = drive;
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        this.handoff = handoff;

        for (String path : pathList) {
            addCommand(path);
        }
    }

    // drive to a pose
    // then pick up and shoot
    // then repeat possibly

    public void addCommand(String path) {
        List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

        double driveTime = Math.max(AutoPathConstants.MAX_ARM_MOVE_TIME, Choreo.getTrajectory(path).getTotalTime());
        double intakeTime = driveTime + AutoPathConstants.INTAKE_TIME;
        double shootTime = intakeTime + AutoPathConstants.MAX_ARM_MOVE_TIME;
        double totalTime = shootTime + AutoPathConstants.SHOOT_TIME;

        eventMarkers.add(
            Pair.of(
                AutoPathConstants.INIT_MOVEMENTS_TIME,
                // turn on handoff
                handoff.getHandoffCommand(AutoPathConstants.HANDOFF_IN_VOLTS)
                // turn on intake (no matter where arm is)
                .alongWith(intake.getIntakeCommand(AutoPathConstants.INTAKE_VOLTS, handoff::checkNoteThere))
                // move arm to intake
                .alongWith(arm.moveArm(ArmPosition.INTAKE, drive::getPose))
                // drive
                .alongWith(drive.followTrajectory(DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path)))
            )
        );

        // move arm to shoot
        eventMarkers.add(Pair.of(intakeTime,
            arm.moveArm(ArmPosition.SHOOT, drive::getPose)));

        // run handoff & shoot
        eventMarkers.add(Pair.of(shootTime,
            handoff.getShootCommand(AutoPathConstants.HANDOFF_OUT_VOLTS,
                shooter::checkNoteShot)));

        command = command == null ? 
            (new AutoCommand(eventMarkers, totalTime)) 
            : command.andThen(new AutoCommand(eventMarkers, totalTime));
    }

    public Command getCommandSequence() {
        return command;
    }
}
