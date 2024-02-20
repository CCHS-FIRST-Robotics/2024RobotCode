package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

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
import frc.robot.subsystems.mecaDrive.Drive;

import com.choreo.lib.*;
import com.ctre.phoenix6.controls.ControlRequest;

public final class MechanismsPath {

    private List<Pair<Double, Command>> eventMarkers;
    private String path;

    public MechanismsPath(String path) {
        this.path = path;
    }

    public void addEventMarker(double time, Command eventCommand) {
        eventMarkers.add(Pair.of(time, eventCommand));
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
