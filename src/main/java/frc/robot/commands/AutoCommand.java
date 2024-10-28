package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.*;

public class AutoCommand extends Command {
    private final Map<Command, Boolean> runningCommands = new HashMap<>();
    private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList<>();
    private List<Pair<Double, Command>> events = new ArrayList<>();
    
    private final Timer timer = new Timer();
    private double totalTime;

    public AutoCommand(List<Pair<Double, Command>> events, double totalTime) {
        this.events = events;
        this.totalTime = totalTime;

        for (Pair<Double, Command> marker : events) {
            Set<Subsystem> reqs = marker.getSecond().getRequirements();
            m_requirements.addAll(reqs);
        }
    }

    @Override
    public void initialize() {
        runningCommands.clear();
        untriggeredEvents.clear();
        untriggeredEvents.addAll(events);

        timer.reset(); // ! might be redundant
        timer.start();
    }

    @Override
    public void execute() {
        if (!untriggeredEvents.isEmpty() && timer.hasElapsed(untriggeredEvents.get(0).getFirst())) {
            Pair<Double, Command> event = untriggeredEvents.remove(0);

            // check and clear any running commands for the same subsystem
            for (Map.Entry<Command, Boolean> runningCommand : runningCommands.entrySet()) {
                if (!runningCommand.getValue()) {
                    continue;
                }

                if (!Collections.disjoint(runningCommand.getKey().getRequirements(), event.getSecond().getRequirements())) {
                    runningCommand.getKey().end(true);
                    runningCommand.setValue(false);
                }
            }

            event.getSecond().initialize();
            runningCommands.put(event.getSecond(), true);
        }

        // execute every running command
        for (Map.Entry<Command, Boolean> runningCommand : runningCommands.entrySet()) {
            if (!runningCommand.getValue()) {
                continue;
            }

            runningCommand.getKey().execute();

            if (runningCommand.getKey().isFinished()) {
                runningCommand.getKey().end(false);
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