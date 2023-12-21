// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.data.CMatrixRMaj;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.DriveInCircle;
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
import frc.robot.utils.DriveTrajectoryGenerator;
import frc.robot.utils.PoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private final PoseEstimator poseEstimator;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandGenericHID wiiRemote1 = new CommandGenericHID(2);
    // private final CommandGenericHID wiiRemote2 = new CommandGenericHID(3);
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
                useWiiRemotes
            );
            camera = new Vision(new CameraIOZED());
            break;

        // Sim robot, instantiate physics sim IO implementations
        case SIM:
            drive = new Drive(
                new GyroIONavX(),
                new ModuleIOSim(), 
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                false
            );
            camera = new Vision(new CameraIOZED());
            break;

        // Replayed robot, disable IO implementations
        default:
            drive = new Drive(
                new GyroIONavX(),
                new ModuleIOSparkMax(0), 
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2), 
                new ModuleIOSparkMax(3),
                false
            );
            camera = new Vision(new CameraIOZED());
            break;
        }

        poseEstimator = new PoseEstimator(
            drive.getKinematics(),
            drive.getPose().getRotation(),
            drive.getModulePositions(),
            drive.getPose()
        );

        drive.setPoseEstimator(poseEstimator);
        camera.setPoseEstimator(poseEstimator);

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
                    () -> {
                        System.out.println(-MathUtil.inputModulus((wiiRemote1.getRawAxis(4) - 1), -1, 1));
                        return -MathUtil.inputModulus((wiiRemote1.getRawAxis(4) - 1), -1, 1);
                    },
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

        // Brake when the left trigger is held
        controller.leftTrigger().whileTrue(
            new RunCommand(drive::stopWithX, drive)
        );

        // controller.rightTrigger().whileTrue(
        //     new RunCommand(drive::runCharacterization, drive)
        // );
        
        // Generate a trajectory to a pose when the A button is pressed (and switch drive to position control)
        controller.a().onTrue(
            new MoveToPose(
                drive, 
                () -> {return new Pose2d(0, 0, new Rotation2d(Math.PI * 3 * .25));}
            )
        );

        // Generate a trajectory to a pose when the X button is pressed (and switch drive to position control
        // new Trigger(() -> {return ((int) Timer.getFPGATimestamp() == 10);}).onTrue(
        controller.x().onTrue(
            drive.runOnce(
                () -> {
                    String path = "RC";
                    var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
                    // adjust so that the start of the trajectory is where the robot is
                    traj.translateBy(traj.positionTrajectory.get(0).getTranslation().unaryMinus());
                    traj.translateBy(drive.getPose().getTranslation()); 

                    System.out.println("Writing trajectory to CSV");
                    traj.toCSV(path);
                    drive.runPosition(traj);
                }
            )
        );

        // controller.b().onTrue(
        //     Commands.runOnce(drive::toggleDriveMotorsBrakeMode)
        // );


        // controller.y().onTrue(
        //     new DriveInCircle(
        //         drive,
        //         () -> {
        //             return getRadius();
        //             // return new Translation2d(.57/2.0, .57/2.0);
        //         },
        //         () -> {
        //             return getVelocity();
        //         },
        //         () -> {
        //             return getAngularVelocity();
        //         }
        //     )
        // );

        controller.y().whileTrue(
            new DriveInCircle(
                drive,
                () -> {
                    // return new Translation2d(2.0, 0.0);
                    return new Translation2d(.57/2.0, .57/2.0);
                },
                () -> {
                    // return 2.5;
                    return 0.75;
                },
                () -> {
                    // return 4 * 2.5 / 2.0;
                    return 0.75 / (new Translation2d(.57/2.0, .57/2.0).getNorm());
                }
            )
        );
    }

    private double applyPreferences(double input, double exponent, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    private Translation2d getRadius() {
        double leftY = applyPreferences(controller.getLeftY(), 2.0, .1);
        return new Translation2d(.5 + 1.5 * Math.abs(leftY), 0.0);
    }

    private double getVelocity() {
        double leftX = applyPreferences(controller.getLeftX(), 2.0, .1);
        return 2 * leftX;
    }

    private double getAngularVelocity() {
        if (getRadius().getNorm() == 0) return 0;
        double rightX = applyPreferences(controller.getRightX(), 2.0, .1);
        return (1 + rightX) * getVelocity() / getRadius().getNorm();
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
