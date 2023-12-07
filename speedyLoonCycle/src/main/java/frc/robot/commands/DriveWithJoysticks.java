package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
public class DriveWithJoysticks extends CommandBase{
    Subsystem subsystem;
    Drive subsystemDrive;
    Supplier<Double> leftX, leftY;
 
    public DriveWithJoysticks(Drive subsystem, Supplier<Double> leftX, Supplier<Double> leftY) {
        this.subsystem = subsystem;
        subsystemDrive = (Drive)subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        this.leftX = leftX;
        this.leftY = leftY;
        // (a) -> getValue(a);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = leftX.get();
        double y = leftY.get();
        subsystemDrive.one.driveMotor.set(y);
        subsystemDrive.one.turnMotor.set(x);
        subsystemDrive.two.driveMotor.set(y);
        subsystemDrive.two.turnMotor.set(x);
        subsystemDrive.three.driveMotor.set(y);
        subsystemDrive.three.turnMotor.set(x);
        subsystemDrive.four.driveMotor.set(y);
        subsystemDrive.four.turnMotor.set(x);
        
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
