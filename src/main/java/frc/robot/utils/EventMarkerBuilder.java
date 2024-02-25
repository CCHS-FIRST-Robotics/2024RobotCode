package frc.robot.utils;

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
import frc.robot.*;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.EventCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class EventMarkerBuilder {

    private List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

    private ArrayList<String> pathList;
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    private Arm arm;

    private ArrayList<Command> commands = new ArrayList<Command>();

    public EventMarkerBuilder(ArrayList<String> pathList, Drive drive, Intake intake, Shooter shooter, Arm arm) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;

        for (String path : pathList) {
            addCommand(path);
        }
    }

    public void addCommand(String path) {
        DriveTrajectory traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);
        double totalTime = choreoTrajectory.getTotalTime();
        List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME, drive.followTrajectory(traj)));
        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME,
                arm.getPosFromPath(path, AutoPathConstants.INIT_MOVEMENTS_TIME)));
        eventMarkers.add(Pair.of(AutoPathConstants.MAX_ARM_MOVE_TIME,
                shooter.getShootNoteCommand(AutoPathConstants.SHOOT_VOLTS)));
        eventMarkers.add(Pair.of(totalTime - AutoPathConstants.MAX_ARM_MOVE_TIME - AutoPathConstants.INTAKE_TIME,
                arm.getMoveAngleCommand(AutoPathConstants.ARM_HANDOFF_ANGLE)));
        eventMarkers.add(Pair.of(totalTime - AutoPathConstants.INTAKE_TIME,
                intake.getHandNoteCommand(AutoPathConstants.INTAKE_HANDOFF_VOLTS)));

        commands.add(new AutoCommand(eventMarkers));
    }

    public ArrayList<Command> getCommands() {
        return commands;
    }
}
