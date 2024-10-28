package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.*;

public class AutoCommand extends Command {

    private final Timer timer = new Timer();
    // private final Supplier<Pose2d> poseSupplier;
    // private final Supplier<ChassisSpeeds> speedsSupplier;
    // private final Consumer<ChassisSpeeds> speedsOutput;

    private double totalTime;

    // For event markers
    private final Map<Command, Boolean> currentEventCommands = new HashMap<>();
    private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList<>();
    private List<Pair<Double, Command>> eventMarkers = new ArrayList<>();

    public AutoCommand(List<Pair<Double, Command>> eventMarkers, double totalTime) {
        // m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.

        this.eventMarkers = eventMarkers;
        this.totalTime = totalTime;

        // add requirements
        for (Pair<Double, Command> marker : eventMarkers) {
            Set<Subsystem> reqs = marker.getSecond().getRequirements();
            m_requirements.addAll(reqs);
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Initialize marker stuff
        currentEventCommands.clear();
        untriggeredEvents.clear();
        untriggeredEvents.addAll(eventMarkers);

        timer.reset();
        timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // checks if the time has reached when an event is supposed to happen
        if (!untriggeredEvents.isEmpty() && timer.hasElapsed(untriggeredEvents.get(0).getFirst())) {
            // Time to trigger this event command
            Pair<Double, Command> event = untriggeredEvents.remove(0);

            // checks is not trying to run a command already running
            for (var runningCommand : currentEventCommands.entrySet()) {
                if (!runningCommand.getValue()) {
                    continue;
                }

                if (!Collections.disjoint(
                        runningCommand.getKey().getRequirements(), event.getSecond().getRequirements())) {
                    runningCommand.getKey().end(true);
                    runningCommand.setValue(false);
                }
            }

            event.getSecond().initialize();
            currentEventCommands.put(event.getSecond(), true);
        }

        // Run event marker commands
        for (Map.Entry<Command, Boolean> runningCommand : currentEventCommands.entrySet()) {
            if (!runningCommand.getValue()) {
                continue;
            }

            runningCommand.getKey().execute();

            if (runningCommand.getKey().isFinished()) {
                runningCommand.getKey().end(false);
                // runningCommand.setValue(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(totalTime);
    }
}