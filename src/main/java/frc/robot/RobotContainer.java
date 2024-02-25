// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import org.ejml.data.CMatrixRMaj;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.*;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
                new ModuleIOSparkMax(3)
            );
            camera = new Vision(new CameraIOZED());
            break;

        // Sim robot, instantiate physics sim IO implementations
        case SIM:
            drive = new Drive(
                new GyroIO() {},
                new ModuleIOSim(), 
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim()
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
                new ModuleIOSparkMax(3)
            );
            camera = new Vision(new CameraIOZED());
            break;
        }

        poseEstimator = new PoseEstimator(
            drive.getKinematics(),
            new Rotation2d(),
            drive.getModulePositions(),
            new Pose2d()
        );

        drive.setPoseEstimator(poseEstimator);
        camera.setPoseEstimator(poseEstimator);

        // Set up auto routines
        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        // Configure the button bindings
        configureButtonBindings();
    }

    public Translation2d getTargetTranslation(Pose3d targetPose) {
        Pose2d currentPose = drive.getPose();

        Translation2d translationToTargetGround = targetPose.getTranslation().toTranslation2d().minus(currentPose.getTranslation());
        return translationToTargetGround;
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
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive, 
                controller::getLeftX, 
                () -> -controller.getLeftY(), 
                () -> -controller.getRightX(), 
                () -> {return 1.0;},
                () -> Rotation2d.fromDegrees(controller.getHID().getPOV())
            )
        );

        // Follow the nearest apriltag while the right trigger is held
        // controller.rightTrigger().whileTrue(new FollowAprilTag(drive, camera));

        // Stop when the left trigger is held (safety stop)
        controller.leftTrigger().whileTrue(
            new RunCommand(drive::stopWithX, drive)
        );

        // Turn on brake vs coast mode
        // controller.b().onTrue(
        //     Commands.runOnce(drive::toggleDriveMotorsBrakeMode)
        // );

        /* 
         * Move drive (future: other subsystems) to track a target
         */
        Pose3d targetPose = new Pose3d(4, 0, 3, new Rotation3d());
        controller.rightTrigger().whileTrue(
            new DriveWithJoysticks(
                drive, 
                controller::getLeftX, 
                () -> -controller.getLeftY(), 
                controller::getRightX, 
                () -> {return 1.0;},
                () -> {
                    Translation2d targetTranslation = getTargetTranslation(targetPose);
                    return targetTranslation.getAngle();
                }
            )
        );
        
        /*
         * Drive Characterization Code
         */ 

        // controller.rightTrigger().whileTrue(
        //     new RunCommand(drive::runCharacterization, drive)
        // );
        // controller.rightTrigger().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
        // controller.rightTrigger().whileTrue(drive.sysIdDynamic(Direction.kForward));
        controller.rightTrigger().whileTrue(drive.sysIdFull());
        
        /* 
         * Drive Auto Code
         */
        // Generate a trajectory to a pose when the A button is pressed (and switch drive to position control)
        controller.a().onTrue(
            new MoveToPose(
                drive, 
                () -> {return new Pose2d(0, 0, new Rotation2d(Math.PI * 3 * .25));}
            )
        );

        // Generate a trajectory to a pose when the X button is pressed (and switch drive to position control)
        // new Trigger(() -> {return ((int) Timer.getFPGATimestamp() == 10);}).onTrue(
        controller.x().onTrue(
            drive.runOnce(
                () -> {
                    String path = "ThirdFloorTest1";
                    var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
                    // adjust so that the start of the trajectory is where the robot is
                    // traj.translateBy(traj.positionTrajectory.get(0).getTranslation().unaryMinus());
                    // traj.translateBy(drive.getPose().getTranslation());

                    System.out.println("recording pos traj");
                    Logger.recordOutput("Auto/GeneratedTrajectory", traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));

                    System.out.println("Writing trajectory to CSV");
                    traj.toCSV(path);
                    drive.runPosition(traj);
                }
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
