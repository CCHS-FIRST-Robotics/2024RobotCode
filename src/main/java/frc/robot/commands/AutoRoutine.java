package frc.robot.commands;

import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.drive.swerveDrive.Drive;
import frc.robot.utils.DriveTrajectory;
import frc.robot.utils.DriveTrajectoryGenerator;
import frc.robot.utils.MechanismsPath;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoRoutine extends Command {

  private final Timer timer = new Timer();
  // private final Supplier<Pose2d> poseSupplier;
  // private final Supplier<ChassisSpeeds> speedsSupplier;
  // private final Consumer<ChassisSpeeds> speedsOutput;
  private final MechanismsPath path;
  Drive drive;
  Intake intake;

  // For event markers
  private final Map<Command, Boolean> currentEventCommands = new HashMap<>();
  private final List<Pair<Double, Command>> untriggeredEvents = new ArrayList<>();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoRoutine(Drive drive, MechanismsPath path) {
    // m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    // this command isn't actully using the drive subsystem, but the command we call later will!
    // so we don't want to add the requirement here or it'll override the requirement of the command we call
    // that probably wasnt a good explanation so just text me if it makes no sense
    // addRequirements(drive);
    this.path = path;

    // add requirements
    for (Pair<Double, Command> marker : path.getEventMarkers()) {
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
    untriggeredEvents.addAll(path.getEventMarkers());

    timer.reset();
    timer.start();

    // runs drive command with path

    // you weren't actually starting this command, just creating it (like for example you dont just do new RunCommand(...), you do Trigger.onTrue(new RunCommand()) - it needs to know when to start)
    // drive.runOnce(
    //     () -> {
    //       var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path.getPath());
    //       drive.runPosition(traj);
    //     });
    // also, we already made this command using the command factory in Drive.java (which also logs the path to akit)
    drive.followTrajectory(
      DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path.getPath())
    ).schedule(); // now it creates the command (returned from the method) and schedules it :)

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}