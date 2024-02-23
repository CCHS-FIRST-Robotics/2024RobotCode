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
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class MechanismsPath {

    private List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();
    private ArrayList<String> path;
    private String currentPath;
    private AutoPathConstants constants;
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    private Arm arm;
    private int currentPathNum = 0;

    /* adds set markers */
    public MechanismsPath(ArrayList<String> path, Drive drive, Intake intake, Shooter shooter, Arm arm) {
        this.path = path;
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;

        currentPath = path.get(currentPathNum);
        drive.updateCurrentAutoPaths(path);
        constants = new AutoPathConstants(drive, intake, shooter, arm);
       
        addConstEventMarkers();
    }

    /* won't add set markers initially */
    public MechanismsPath(ArrayList<String> path) {
        this.path = path;
    }

    public void addEventMarker(double time, Command eventCommand) {
        eventMarkers.add(Pair.of(time, eventCommand));
        // time in seconds
    }

    public void addConstEventMarkers() {
        for (Map.Entry<Pair<Double,Integer>, ArrayList<String>> em : constants.eventMarkerMap.entrySet()) {
            if (em.getValue().equals(path)) {
                eventMarkers.add(Pair.of(em.getKey().getFirst(), getCommand(em.getKey().getSecond())));
            }
        }
        // time in seconds
    }

    public void updateCurrentPath() {
        currentPathNum++;
        currentPath = path.get(currentPathNum);
    }

    public Command getCommand(int commandNum) {
        switch (commandNum) {
            case AutoPathConstants.INTAKE:
                return intake.getIntakeCommand(AutoPathConstants.INTAKE_VOLTS);
            case AutoPathConstants.SHOOT:
                return shooter.getShootNoteCommand(AutoPathConstants.SHOOT_VOLTS);
            case AutoPathConstants.INTAKE_HANDOFF:
                return intake.getHandNoteCommand(AutoPathConstants.INTAKE_HANDOFF_VOLTS);
            case AutoPathConstants.SHOOTER_HANDOFF:
                return shooter.getReceiveNoteCommand(AutoPathConstants.SHOOTER_HANDOFF_VOLTS);
            case AutoPathConstants.DRIVE_PATH:
                return drive.autoFollowTrajectory();
            case AutoPathConstants.ARM_SHOOT:
                return arm.autoAlignWithTarget();
            case AutoPathConstants.ARM_HANDOFF:
                return arm.alignForHandoff();
            default:
                return intake.runOnce(() -> {}); //idk
        }
    }

    public List<Pair<Double, Command>> getEventMarkers(){
        // put them in time order here
        return eventMarkers;
    }

    // public String getPath() {
    //     return path;
    // }

    public Drive getDrive() {
        return drive;
    }

    public Intake getIntake() {
        return intake;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Arm getArm() {
        return arm;
    }
}
