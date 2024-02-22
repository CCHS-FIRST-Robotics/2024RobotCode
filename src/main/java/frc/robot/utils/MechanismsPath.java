package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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
    private String path;
    private AutoPathConstants constants;

    /* adds set markers */
    public MechanismsPath(String path, Drive drive, Intake intake, Shooter shooter, Arm arm) {
        this.path = path;
        constants = new AutoPathConstants(drive, intake, shooter, arm);
        addConstEventMarkers();
    }

    /* won't add set markers intially */
    public MechanismsPath(String path) {
        this.path = path;
    }

    public void addEventMarker(double time, Command eventCommand) {
        eventMarkers.add(Pair.of(time, eventCommand));
        // time in seconds
    }

    public void addConstEventMarkers() {
        for (Map.Entry<Pair<Double,Command>, String> em : constants.eventMarkerMap.entrySet()) {
            if (em.getValue().equals(path)) {
                eventMarkers.add(em.getKey());
            }
        }
        // time in seconds
    }

    public List<Pair<Double, Command>> getEventMarkers(){
        // put them in time order here
        return eventMarkers;
    }

    public String getPath() {
        return path;
    }
}
