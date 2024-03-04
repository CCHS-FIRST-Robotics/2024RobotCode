// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.DriveWithJoysticks;
// import frc.robot.commands.DriveWithWiimote;
// import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
import frc.robot.commands.DriveInCircle;
import frc.robot.commands.DriveModules;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithWiimote;
import frc.robot.commands.FollowAprilTag;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.AutoRoutine;
import frc.robot.Constants.AutoPathConstants;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.swerveDrive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.utils.EventMarkerBuilder;
import frc.robot.utils.MechanismsPath;
import frc.robot.utils.PoseEstimator;
import frc.robot.subsystems.noteIO.arm.*;
import frc.robot.subsystems.noteIO.intakeArm.*;
import frc.robot.subsystems.noteIO.shooter.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

@SuppressWarnings("unused")
public class RobotContainer {
    private final Drive drive;
    private final Vision camera;
    private final PoseEstimator poseEstimator;

    private final Arm arm;
    private final IntakeArm intake;
    private final Shooter shooter;

    private final CommandXboxController controller = new CommandXboxController(0);
    // private final CommandGenericHID wiiRemote = new CommandGenericHID(2);
    private final boolean useWiiRemotes = false;

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSparkMax(0),
                        new ModuleIOSparkMax(1),
                        new ModuleIOSparkMax(2),
                        new ModuleIOSparkMax(3),
                        useWiiRemotes);
                camera = new Vision(new CameraIOZED());
                arm = new Arm(new ArmIOFalcon500(20, 19));
                intake = new IntakeArm(new IntakeArmIOFalcon500(Constants.INTAKE_ID));
                shooter = new Shooter(new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
            case SIM:
                // instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        false);
                camera = new Vision(new CameraIOZED());
                arm = new Arm(new ArmIOSim());
                intake = new IntakeArm(new IntakeIOSim());
                shooter = new Shooter(new ShooterIOSim());
                break;
            default: // replayed robot
                // disable IO implementations
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSparkMax(0),
                        new ModuleIOSparkMax(1),
                        new ModuleIOSparkMax(2),
                        new ModuleIOSparkMax(3),
                        false);
                camera = new Vision(new CameraIOZED());
                arm = new Arm(new ArmIOFalcon500(20, 19));
                intake = new IntakeArm(new IntakeArmIOFalcon500(Constants.INTAKE_ID));
                shooter = new Shooter(new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
        }

        poseEstimator = new PoseEstimator(
            drive.getKinematics(),
            new Rotation2d(),
            drive.getModulePositions(),
            new Pose2d(
                1.4318007230758516,
                5.557066917419431,
                new Rotation2d()
            )
        );

        drive.setPoseEstimator(poseEstimator);
        camera.setPoseEstimator(poseEstimator);

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand()); // set up autoroutines

        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // // using joysticks
        // drive.setDefaultCommand(
        // new DriveWithJoysticks(
        // drive,
        // controller::getLeftX,
        // () -> -controller.getLeftY(),
        // () -> -controller.getRightX(),
        // () -> {
        // return 1.0;
        // },
        // () -> Rotation2d.fromDegrees(controller.getHID().getPOV())));

        // // break when leftTrigger is held
        // controller.leftTrigger().whileTrue(new RunCommand(drive::stopWithX, drive));

        // outtake
        controller.x().whileTrue(new StartEndCommand(() -> intake.start(Volts.of(-2.9)), () -> intake.stop(), intake));

        // manual intake
        controller.y().whileTrue(new StartEndCommand(() -> intake.start(Volts.of(4)), () -> intake.stop(), intake));

        // intake (stops automatically)
        controller.a().onTrue(intake.getIntakeCommand(Volts.of(2.9)));

        // // intake and move arm (stops automatically)
        // controller.a().onTrue(
        // new InstantCommand(() -> arm.setArmAngle(Degrees.of(90)))
        // .andThen(intake.getIntakeCommand(2.9)));

        // // shoot with arm
        // Generate a trajectory to a pose when the X button is pressed (and switch drive to position control)
        // String path = AutoPathConstants.THREE_NOTE_WING;
        new Trigger(() -> {return ((int) Timer.getFPGATimestamp() == 45);}).onTrue(
        // controller.x().onTrue(
            // new AutoRoutine(new MechanismsPath(AutoPathConstants.threeNoteWing, drive, intake, shooter, arm))
            new EventMarkerBuilder(AutoPathConstants.threeNoteWingSplits, drive, intake, shooter, arm).getCommandSequence()

            // drive.runOnce(
            //     () -> {
                    
            //         var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
            //         // adjust so that the start of the trajectory is where the robot is
            //         // traj.translateBy(traj.positionTrajectory.get(0).getTranslation().unaryMinus());
            //         // traj.translateBy(drive.getPose().getTranslation());

            //         System.out.println("recording pos traj");
            //         Logger.recordOutput("Auto/GeneratedTrajectory", traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));

            //         System.out.println("Writing trajectory to CSV");
            //         traj.toCSV(path);
            //         drive.runPosition(traj);
            //     }
            // ).asProxy().andThen(new AutoRoutine(drive, new MechanismsPath(path, intake)))
        );

        // controller.b().onTrue(
        // // move arm
        // new InstantCommand(() -> arm.setArmAngle(Degrees.of(10))).andThen(
        // // prime shooter
        // new InstantCommand(() -> shooter.start(4), shooter)
        // // wait until shooter is up to speed
        // .alongWith(Commands.waitUntil(shooter::upToSpeed)))

        // shoot
        controller.b().onTrue(
                // prime shooter
                new InstantCommand(() -> shooter.start(RotationsPerSecond.of(10)), shooter)
                        // wait until shooter is up to speed
                        .alongWith(Commands.waitUntil(shooter::upToSpeed))
                        // shoot
                        .andThen(intake.getShootCommand(Volts.of(8), shooter::checkNoteShot))
                        // stop shooter
                        .andThen(new InstantCommand(shooter::stop, shooter)));

        // // drive to specific pose
        // Pose3d targetPose = new Pose3d(4, 0, 3, new Rotation3d());
        // controller.rightTrigger().whileTrue(
        // new DriveWithJoysticks(
        // drive,
        // controller::getLeftX,
        // () -> -controller.getLeftY(),
        // controller::getRightX,
        // () -> {
        // return 1.0;
        // },
        // () -> {
        // Translation2d targetTranslation = getTargetTranslation(targetPose);
        // return targetTranslation.getAngle();
        // }));

        // // create a trajectory to a specific pose
        // controller.a().onTrue(
        // new MoveToPose(
        // drive,
        // () -> {
        // return new Pose2d(0, 0, new Rotation2d(Math.PI * 3 * .25));
        // }));

        // new Trigger(() -> {return ((int) Timer.getFPGATimestamp() == 10);}).onTrue(
        // controller.x().onTrue(
        // drive.runOnce(
        // () -> {
        // String path = "ThirdFloorTest1";
        // var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
        // traj.translateBy(traj.positionTrajectory.get(0).getTranslation().unaryMinus());

        // // follow nearest aprilTag when rightTrigger is held
        // controller.rightTrigger().whileTrue(new FollowAprilTag(drive, camera));

        // // record robot trajectory
        // System.out.println("recording pos traj");
        // Logger.recordOutput("Auto/GeneratedTrajectory",
        // traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));

        // System.out.println("Writing trajectory to CSV");
        // traj.toCSV(path);
        // drive.runPosition(traj);

        // // sysID
        // controller.rightTrigger().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
        // controller.rightTrigger().whileTrue(drive.sysIdDynamic(Direction.kForward));
        // controller.rightTrigger().whileTrue(drive.sysIdFull());
    }

    public Translation2d getTargetTranslation(Pose3d targetPose) {
        Pose2d currentPose = drive.getPose();
        Translation2d translationToTargetGround = targetPose.getTranslation()
                .toTranslation2d()
                .minus(currentPose.getTranslation());
        return translationToTargetGround;
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