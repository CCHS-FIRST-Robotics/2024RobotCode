package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.utils.EventMarkerBuilder;

public class AutoCommandBuilder extends Command {

  private ArrayList<Command> commandList = new ArrayList<Command>();

  public AutoCommandBuilder(ArrayList<Command> commandList) {

    this.commandList = commandList;

    // add requirements
    for (Command command : commandList) {
        Set<Subsystem> reqs = command.getRequirements();
      m_requirements.addAll(reqs);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (Command command : commandList) {
        command.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}