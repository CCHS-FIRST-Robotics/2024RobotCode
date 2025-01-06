package frc.robot.utils;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.Pair;
import com.choreo.lib.*;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.Drive;

public final class AutoCommandSequenceBuilder {
    private Command autoCommandSequence;
    
    private Drive drive;

    public AutoCommandSequenceBuilder(
        ArrayList<String> pathList, 
        Drive drive
    ){
        this.drive = drive;

        // append one AutoCommand per path section to the sequence
        for (String path : pathList) {
            addCommand(path);
        }
    }

    public void addCommand(String path) {
        List<Pair<Double, Command>> events = new ArrayList<Pair<Double, Command>>();

        events.add(Pair.of(0.0, drive.followTrajectory(DriveTrajectoryGenerator.generateChoreoTrajectory(path))));
        double totalTime =  Choreo.getTrajectory(path).getTotalTime();

        // append to the command sequence
        autoCommandSequence = autoCommandSequence == null ? 
            (new AutoCommand(events, totalTime)) 
            : autoCommandSequence.andThen(new AutoCommand(events, totalTime)
        );
    }

    public Command getAutoCommandSequence() {
        return autoCommandSequence;
    }
}