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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.mecaDrive.Drive;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class MechanismsPath {

    private List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();
    private String path;
    private AutoPathConstants constants;

    public MechanismsPath(String path, Intake intake) {
        this.path = path;
        constants = new AutoPathConstants(intake);
        addEventMarkers();
    }

    public void addEventMarkers() {
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
