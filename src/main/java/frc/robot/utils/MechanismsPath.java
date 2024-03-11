package frc.robot.utils;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.intakeGround.IntakeGround;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import edu.wpi.first.units.Units.*;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class MechanismsPath {

    private List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();
    private ArrayList<String> path;
    private Drive drive;
    private IntakeGround intake;
    private Shooter shooter;
    private Arm arm;
   

    /* adds set markers */
    public MechanismsPath(ArrayList<String> path, Drive drive, IntakeGround intake, Shooter shooter, Arm arm) {
        this.path = path;
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm; 
       
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
        for (Map.Entry<Pair<Double,Constants.EventCommand>, ArrayList<String>> em : AutoPathConstants.eventMarkerMap.entrySet()) {
            if (em.getValue().equals(path)) {
                eventMarkers.add(Pair.of(em.getKey().getFirst(), getCommand(em.getKey().getSecond(), em.getKey().getFirst())));
            }
        }
        // time in seconds
    }

    public Command getCommand(Constants.EventCommand commandNum, double time) {
        switch (commandNum) {
            case INTAKE:
                return intake.getIntakeCommand(AutoPathConstants.INTAKE_VOLTS);
            case SHOOT:
                return shooter.getShootNoteCommand(RadiansPerSecond.of(AutoPathConstants.SHOOT_VOLTS));
            case INTAKE_HANDOFF:
                return intake.getHandNoteCommand(AutoPathConstants.INTAKE_HANDOFF_VOLTS);
            case SHOOTER_HANDOFF:
                return shooter.getReceiveNoteCommand(RadiansPerSecond.of(AutoPathConstants.SHOOTER_HANDOFF_VOLTS));
            case DRIVE_PATH:
                // return drive.followTrajectory(path); for split
                return drive.followTrajectory(DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path.get(0)));
            case ARM_SHOOT:
                return arm.moveArm(ArmPosition.SHOOT, arm.getPosFromPath(path.get(0), time));
            case ARM_HANDOFF:
                // return arm.getMoveAngleCommand(AutoPathConstants.ARM_HANDOFF_ANGLE);
                // return arm.getMoveAngleCommand(AutoPathConstants.QUOKKA_ARM_INTAKE_ANGLE);
                return arm.moveArm(ArmPosition.INTAKE, arm.getPosFromPath(path.get(0), time));
            default:
                return null;
        }
    }

    public List<Pair<Double, Command>> getEventMarkers(){
        // put them in time order here
        return eventMarkers;
    }
}
