// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.data.CMatrixRMaj;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.DriveModules;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithWiimote;
import frc.robot.commands.FollowAprilTag;
import frc.robot.commands.MoveToPose;

// import frc.robot.subsystems.mecaDrive.Drive;
// import frc.robot.subsystems.mecaDrive.DriveIO;
// import frc.robot.subsystems.mecaDrive.DriveIOSim;
// import frc.robot.subsystems.mecaDrive.DriveIOSparkMax;

import frc.robot.subsystems.swerveDrive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.utils.PoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision camera;
    private final PoseEstimator poseEstimator = new PoseEstimator();

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandGenericHID wiiRemote1 = new CommandGenericHID(2);
    private final CommandGenericHID wiiRemote2 = new CommandGenericHID(3);
    private final boolean useWiiRemotes = false;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
        case REAL:
            drive = new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(0), 
                new ModuleIOSparkMax(1), 
                new ModuleIOSparkMax(2), 
                new ModuleIOSparkMax(3),
                poseEstimator,
                useWiiRemotes
            );
            camera = new Vision(new CameraIOZED(), poseEstimator);
            break;

        // Sim robot, instantiate physics sim IO implementations
        case SIM:
            drive = new Drive(
                new GyroIONavX(),
                new ModuleIOSim(), 
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                poseEstimator,
                false
            );
            camera = new Vision(new CameraIOZED(), poseEstimator);
            break;

        // Replayed robot, disable IO implementations
        default:
            drive = new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(0), 
                new ModuleIOSparkMax(1), 
                new ModuleIOSparkMax(2), 
                new ModuleIOSparkMax(3),
                poseEstimator,
                false
            );
            camera = new Vision(new CameraIOZED(), poseEstimator);
            break;
        }

        // Set up auto routines
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        // Configure the button bindings
        configureButtonBindings();
    }

    private Rotation2d getWiiPOV() {
        double povX = wiiRemote1.getRawAxis(0); // should be binary (-1, 0, or 1)
        double povY = wiiRemote1.getRawAxis(1);
        if (Math.abs(povX) < .05) povX = 0; // deadband since 0 isnt 0 for some reason
        if (Math.abs(povY) < .05) povY = 0;

        if (povX == 0 && povY == 0) return new Rotation2d(-1);
        double povAngle = Math.atan2(povY, povX);
        return new Rotation2d(povAngle);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // DRIVING MODULES -- FOR TESTING
        // drive.setDefaultCommand(
        //   new DriveModules(
        //     drive, 
        //     () -> -controller.getLeftY(), 
        //     () -> controller.getRightX(), 
        //     () -> 0.5 + 0.5 * controller.getRightTriggerAxis()
        // ));

        // DRIVING WITH JOYSTICKS (NORMAL)
        if (useWiiRemotes) {
            drive.setDefaultCommand(
                new DriveWithWiimote(
                    drive,
                    () -> wiiRemote1.getRawAxis(3),
                    () -> wiiRemote1.getRawAxis(4),
                    wiiRemote1.button(1),
                    wiiRemote1.button(2),
                    () -> getWiiPOV(),
                    () -> {return 1.0;}
                )
            );
        } else {
            drive.setDefaultCommand(
                new DriveWithJoysticks(
                    drive, 
                    () -> controller.getLeftX(), 
                    () -> -controller.getLeftY(), 
                    () -> controller.getRightX(), 
                    () -> {return 1.0;}
                )
            );
        }

        // Follow the nearest apriltag while the right trigger is held
        controller.rightTrigger().whileTrue(new FollowAprilTag(drive, camera));
        
        // Generate a trajectory to a pose when the A button is pressed (and switch drive to position control)
        controller.a().onTrue(
            new MoveToPose(
                drive, 
                () -> {return new Pose2d(-1, -1, new Rotation2d(0));}
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
