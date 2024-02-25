// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithWiimote;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.swerveDrive.*;
import frc.robot.subsystems.vision.*;
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
public class RobotContainer {
    private final Drive drive;
    private final Vision camera;
    private final PoseEstimator poseEstimator;

    private final Arm arm;
    private final IntakeArm intake;
    private final Shooter shooter;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandGenericHID wiiRemote1 = new CommandGenericHID(2);
    private final boolean useWiiRemotes = false;

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // creating the drivebase
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
                break;
        }

        poseEstimator = new PoseEstimator(
                drive.getKinematics(),
                new Rotation2d(),
                drive.getModulePositions(),
                new Pose2d());
        drive.setPoseEstimator(poseEstimator);
        camera.setPoseEstimator(poseEstimator);

        arm = new Arm(new ArmIOFalcon500(100, 100));
        intake = new IntakeArm(new IntakeArmIOFalcon500(Constants.INTAKE_ID));
        shooter = new Shooter(new ShooterIOCIM(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));

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
        if (!useWiiRemotes) {
            // using joysticks
            drive.setDefaultCommand(
                    new DriveWithJoysticks(
                            drive,
                            controller::getLeftX,
                            () -> -controller.getLeftY(),
                            () -> -controller.getRightX(),
                            () -> {
                                return 1.0;
                            },
                            () -> Rotation2d.fromDegrees(controller.getHID().getPOV())));
        } else {
            // using wii remote
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
                            () -> {
                                return 1.0;
                            }));
        }

        // break when leftTrigger is held
        controller.leftTrigger().whileTrue(new RunCommand(drive::stopWithX, drive));

        // outtake
        controller.x().whileTrue(new StartEndCommand(() -> intake.start(-6), () -> intake.stop(), intake));

        // intake
        controller.a().onTrue(
                // position the arm
                new InstantCommand(() -> arm.setArmAngle(Radians.of(10)))
                        // run intake
                        .andThen(intake.getIntakeCommand(10)));

        // shoot
        controller.b().onTrue(
                // position the arm
                new InstantCommand(() -> arm.setArmAngle(Radians.of(100)))
                        // prime shooter
                        .andThen(new InstantCommand(() -> shooter.start(10), shooter))
                        // shoot
                        .andThen(intake.getShootCommand(10, shooter::checkNoteShot)
                                // stop shooter
                                .andThen(new InstantCommand(() -> shooter.stop(), shooter))));

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

        // // drive in a circle
        // controller.y().onTrue(
        // new DriveInCircle(
        // drive,
        // () -> {
        // return getRadius();
        // },
        // () -> {
        // return getVelocity();
        // },
        // () -> {
        // return getAngularVelocity();
        // }));
        // controller.y().whileTrue(
        // new DriveInCircle(
        // drive,
        // () -> {
        // return new Translation2d(.57 / 2.0, .57 / 2.0);
        // },
        // () -> {
        // return 0.75;
        // },
        // () -> {
        // return 0.75 / (new Translation2d(.57 / 2.0, .57 / 2.0).getNorm());
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

    private Rotation2d getWiiPOV() {
        double povX = wiiRemote1.getRawAxis(0); // should be binary (-1, 0, or 1)
        double povY = wiiRemote1.getRawAxis(1);
        if (Math.abs(povX) < .05)
            povX = 0; // deadband since 0 isnt 0 for some reason
        if (Math.abs(povY) < .05)
            povY = 0;

        if (povX == 0 && povY == 0)
            return new Rotation2d(-1);
        double povAngle = Math.atan2(povY, povX);
        return new Rotation2d(povAngle);
    }

    // public Translation2d getTargetTranslation(Pose3d targetPose) {
    // Pose2d currentPose = drive.getPose();
    // Translation2d translationToTargetGround = targetPose.getTranslation()
    // .toTranslation2d()
    // .minus(currentPose.getTranslation());
    // return translationToTargetGround;
    // }

    // private Translation2d getRadius() {
    // double leftY = applyPreferences(controller.getLeftY(), 2.0, .1);
    // return new Translation2d(.5 + 1.5 * Math.abs(leftY), 0.0);
    // }

    // private double getVelocity() {
    // double leftX = applyPreferences(controller.getLeftX(), 2.0, .1);
    // return 2 * leftX;
    // }

    // private double getAngularVelocity() {
    // if (getRadius().getNorm() == 0)
    // return 0;
    // double rightX = applyPreferences(controller.getRightX(), 2.0, .1);
    // return (1 + rightX) * getVelocity() / getRadius().getNorm();
    // }

    // private double applyPreferences(double input, double exponent, double
    // deadzone) {
    // if (Math.abs(input) < deadzone) {
    // return 0;
    // }
    // return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}