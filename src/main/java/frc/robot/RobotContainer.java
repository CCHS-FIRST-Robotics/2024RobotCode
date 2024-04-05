// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser; // ! yeah do something with this
import frc.robot.commands.*;
import frc.robot.utils.PoseEstimator;
import frc.robot.subsystems.drive.swerveDrive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.noteIO.arm.*;
import frc.robot.subsystems.noteIO.handoff.*;
import frc.robot.subsystems.noteIO.intake.*;
import frc.robot.subsystems.noteIO.shooter.*;
import frc.robot.utils.EventMarkerBuilder;
import static frc.robot.Constants.*;

import java.util.function.Supplier; // !!!!!!!!!!!!

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Vision vision;

    private final Arm arm;
    private final Handoff handoff;
    private final Intake intake;
    private final Shooter shooter;

    private final CommandXboxController controller1 = new CommandXboxController(Constants.CONTROLLER_PORT_1);
    private final CommandXboxController controller2 = new CommandXboxController(Constants.CONTROLLER_PORT_2);

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSparkMax(0),
                        new ModuleIOSparkMax(1),
                        new ModuleIOSparkMax(2),
                        new ModuleIOSparkMax(3));
                vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());
                arm = new Arm(
                        new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID,
                                Constants.ARM_CANCODER_ID));
                intake = new Intake(
                        new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
                handoff = new Handoff(
                        // new HandoffIOFalcon500(Constants.HANDOFF_ID)
                        new HandoffIOSparkMax(Constants.HANDOFF_ID));
                shooter = new Shooter(
                        new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
            case SIM:
                // instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());
                arm = new Arm(new ArmIOSim());
                handoff = new Handoff(new HandoffIOSim());
                intake = new Intake(new IntakeIOSim()); //////// change
                shooter = new Shooter(new ShooterIOSim());
                break;
            default: // replayed robot; disable IO implementations
                drive = new Drive(
                        new GyroIONavX(),
                        new ModuleIOSparkMax(0),
                        new ModuleIOSparkMax(1),
                        new ModuleIOSparkMax(2),
                        new ModuleIOSparkMax(3));
                vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());
                arm = new Arm(
                        new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID,
                                Constants.ARM_CANCODER_ID));
                intake = new Intake(
                        new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
                handoff = new Handoff(new HandoffIOFalcon500(Constants.HANDOFF_ID));
                shooter = new Shooter(
                        new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
        }

        // ! wow this looks like it should be automized
        // change pose here for autos!!!
        poseEstimator = new PoseEstimator(
                drive.getKinematics(),
                new Rotation2d(),
                drive.getModulePositions(),
                StartPosistions.blueCenter);

        drive.setPoseEstimator(poseEstimator);
        vision.setPoseEstimator(poseEstimator);

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand()); // set up autoroutines

        configureButtonBindings();
    }

    // ! ask colin tf is this for
    public void switchDriveThing() {
        System.out.println("ahahhhhhhh");
        drive.removeDefaultCommand();
        drive.setDefaultCommand(
                new DriveWithJoysticks(
                        drive,
                        () -> -controller1.getLeftX(),
                        () -> -controller1.getLeftY(),
                        () -> -.7 * controller1.getRightX(),
                        () -> {
                            return 1.0;
                        },
                        // () -> {return new Rotation2d();},
                        () -> Rotation2d.fromDegrees(controller1.getHID().getPOV()),
                        false,
                        true));
    }

    // ! this too
    public void switchDriveThing2() {
        drive.removeDefaultCommand();
        drive.setDefaultCommand(
                new DriveWithJoysticks(
                        drive,
                        () -> controller1.getLeftX(),
                        () -> controller1.getLeftY(),
                        () -> -.7 * controller1.getRightX(),
                        () -> {
                            return 1.0;
                        },
                        // () -> {return new Rotation2d();},
                        () -> Rotation2d.fromDegrees(controller1.getHID().getPOV()),
                        false,
                        true));
    }

    private void configureButtonBindings() {
        // ! uhhhhhhhhhhhhhh (basically run an auto during teleop)
        controller1.x().onTrue(
                new EventMarkerBuilder(AutoPathConstants.fourC231, drive, intake, handoff, shooter, arm)
                        .getCommandSequence());

        /*
         * Controller 1:
         * - Drive
         * - Shoot
         */

        // drive with joysticks
        drive.setDefaultCommand(
                new DriveWithJoysticks(
                        drive,
                        () -> -controller1.getLeftX(),
                        () -> -controller1.getLeftY(),
                        () -> -.7 * controller1.getRightX(),
                        () -> {
                            return 1.0;
                        },
                        // () -> {return new Rotation2d();},
                        () -> Rotation2d.fromDegrees(controller1.getHID().getPOV()),
                        false,
                        true));

        // shoot
        controller1.b().onTrue(
                // handoff
                handoff.getShootCommand(Volts.of(12), shooter::checkNoteShot)
                        // stop shooter
                        .andThen(new InstantCommand(shooter::stop, shooter)));

        /*
         * Controller 2:
         * - Intake / Outtake
         * - Prime shooter
         */

        // intake
        controller2.a().onTrue(
                // turn on handoff
                handoff.getHandoffCommand(Volts.of(3))
                        // move arm down
                        .alongWith(arm.moveArm(ArmPosition.INTAKE, drive::getPose)
                                .andThen(Commands.waitUntil(arm::atGoal)))
                        .alongWith(
                                // turn on intake until detected by handoff
                                intake.getIntakeCommand(Volts.of(8), handoff::checkNoteThere)));

        // outtake
        controller2.x().whileTrue(
                intake.getIntakeCommand(Volts.of(-3))
                        .alongWith(handoff.getHandoffManualCommand(Volts.of(-2))));

        // ! move to drive subsystem
        Supplier<Rotation2d> shootHeading = () -> drive.getPose().getTranslation()
                .minus(SPEAKER_POSE.getTranslation()).getAngle()
                .plus(new Rotation2d(Math.PI));

        /**
         * Priming the shooter
         */

        // prime shooter (amp)
        controller2.y().and(() -> !shooter.upToSpeed()).onTrue(
                // prime shooter
                new InstantCommand(() -> shooter.start(SHOOTER_AMP_SPEED, SHOOTER_AMP_SPEED), shooter)
                        // move arm
                        .alongWith(arm.moveArm(ArmPosition.AMP, drive::getPose)));

        // prime shooter (speaker - anywhere)
        controller2.b().and(() -> !shooter.upToSpeed()).onTrue(
                // prime shooter
                new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
                        // move arm
                        .alongWith(arm.moveArm(ArmPosition.SHOOT, drive::getPose))
                        // turn robot towards speaker
                        .alongWith(
                                new DriveWithJoysticks(
                                        drive,
                                        () -> -controller1.getLeftX(),
                                        () -> -controller1.getLeftY(),
                                        () -> -.7 * controller1.getRightX(),
                                        () -> {
                                            return 1.0;
                                        },
                                        shootHeading,
                                        true,
                                        true)));

        // prime shooter (speaker - subwoofer)
        controller2.povRight().and(() -> !shooter.upToSpeed()).onTrue(
                // prime shooter
                new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
                        // move arm
                        .alongWith(arm.moveArm(ArmPosition.SPEAKER, drive::getPose)));

        // prime shooter (speaker - close to sub)
        controller2.povUp().and(() -> !shooter.upToSpeed()).onTrue(
                // prime shooter
                new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
                        // move arm
                        .alongWith(arm.moveArm(ArmPosition.CLOSE_SUB, drive::getPose)));

        // prime shooter (speaker - stage)
        controller2.povLeft().and(() -> !shooter.upToSpeed()).onTrue(
                // prime shooter
                new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
                        // move arm
                        .alongWith(arm.moveArm(ArmPosition.STAGE, drive::getPose)));

        // arm position testing
        // controller1.leftTrigger().onTrue(new InstantCommand(() -> {
        // ARM_POSITIONS.put(ArmPosition.TEST,
        // ARM_POSITIONS.get(ArmPosition.TEST).minus(Degrees.of(.5)));
        // arm.setArmAngle(ARM_POSITIONS.get(ArmPosition.TEST));
        // }));
        // controller1.rightTrigger().onTrue(new InstantCommand(() -> {
        // ARM_POSITIONS.put(ArmPosition.TEST,
        // ARM_POSITIONS.get(ArmPosition.TEST).plus(Degrees.of(.5)));
        // arm.setArmAngle(ARM_POSITIONS.get(ArmPosition.TEST));
        // }));
        // controller1.a().onTrue(new InstantCommand(() -> {
        // System.out.println("ARM ANGLE IS : " + ARM_POSITIONS.get(ArmPosition.TEST));
        // }));
    }

    // ! probably unused
    public Translation2d getTargetTranslation(Pose3d targetPose) {
        Pose2d currentPose = drive.getPose();
        Translation2d translationToTargetGround = targetPose.getTranslation()
                .toTranslation2d()
                .minus(currentPose.getTranslation());
        return translationToTargetGround;
    }

    // ! look at this later
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new EventMarkerBuilder(AutoPathConstants.fourC231, drive, intake, handoff, shooter, arm)
                .getCommandSequence();
        // Command command = new InstantCommand(() ->
        // shooter.start(AutoPathConstants.SHOOT_SPEED_LEFT,
        // AutoPathConstants.SHOOT_SPEED_RIGHT), shooter)
        // .andThen(
        // arm.moveArm(ArmPosition.SPEAKER, drive::getPose)
        // .alongWith(
        // Commands.waitUntil(() -> shooter.upToSpeed() && arm.isAtGoal())
        // .andThen(handoff.getShootCommand(AutoPathConstants.HANDOFF_IN_VOLTS,
        // shooter::checkNoteShot))
        // ).until(shooter::checkNoteShot)
        // );

        // command = command.andThen(
        // new InstantCommand(() -> shooter.stop())
        // .alongWith(new InstantCommand(() -> handoff.stop())));
        // return command;
    }
}