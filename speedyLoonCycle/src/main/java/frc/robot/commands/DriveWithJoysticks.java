package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
public class DriveWithJoysticks extends CommandBase{
    Subsystem subsystem;
    public DriveWithJoysticks(Drive subsystem) {
        this.subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
}
