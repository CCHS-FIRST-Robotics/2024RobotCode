package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
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

    private Command command;

    public EventMarkerBuilder(ArrayList<String> pathList, Drive drive, IntakeGround intake, IntakeArm handoff,
            Shooter shooter, Arm arm) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
        this.handoff = handoff;

        for (String path : pathList) {
            addCommand(path);
        }
    } 

    public void addCommand(String path) {
        DriveTrajectory traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path);
        double totalTime = choreoTrajectory.getTotalTime() + AutoPathConstants.SHOOT_TIME + AutoPathConstants.SHOOT_TIME; // + time to shoot
        List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

        // arm shoot
        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME,
        arm.moveArm(ArmPosition.SHOOT, arm.getPosFromPath(path, AutoPathConstants.INIT_MOVEMENTS_TIME))));
        // shoot
        eventMarkers.add(Pair.of(AutoPathConstants.MAX_ARM_MOVE_TIME,
        shooter.getReceiveNoteCommand(RadiansPerSecond.of(AutoPathConstants.SHOOTER_HANDOFF_VOLTS))));
        // drive
        eventMarkers.add(Pair.of(AutoPathConstants.MAX_ARM_MOVE_TIME + AutoPathConstants.SHOOT_TIME, drive.followTrajectory(traj)));
        // arm hand
        eventMarkers.add(Pair.of((totalTime - AutoPathConstants.MAX_ARM_MOVE_TIME - AutoPathConstants.INTAKE_TIME),
                arm.moveArm(ArmPosition.INTAKE, arm.getPosFromPath(path,
                        totalTime - AutoPathConstants.MAX_ARM_MOVE_TIME - AutoPathConstants.INTAKE_TIME))));
        // intake
        eventMarkers.add(Pair.of((totalTime - AutoPathConstants.INTAKE_TIME),
                intake.getIntakeCommand(AutoPathConstants.INTAKE_HANDOFF_VOLTS)));
        // handoff
        eventMarkers.add(Pair.of((totalTime - AutoPathConstants.INTAKE_TIME),
                handoff.getHandoffCommand(Volt.of(AutoPathConstants.INTAKE_HANDOFF_VOLTS))));

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
