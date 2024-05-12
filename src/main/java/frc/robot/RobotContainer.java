// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.AutoPathConstants;
import frc.robot.Constants.StartPosistions;
import frc.robot.commands.*;
import frc.robot.utils.PoseEstimator;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.noteIO.arm.*;
import frc.robot.subsystems.noteIO.handoff.*;
import frc.robot.subsystems.noteIO.intake.*;
import frc.robot.subsystems.noteIO.shooter.*;
import frc.robot.utils.EventMarkerBuilder;
import static frc.robot.Constants.*;

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

    // ! yeah do something with this
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
                arm = new Arm(new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID, Constants.ARM_CANCODER_ID));
                intake = new Intake(new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
                handoff = new Handoff(new HandoffIOSparkMax(Constants.HANDOFF_ID));
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
                        new ModuleIOSim());
                vision = new Vision(new CameraIOZED(), new CameraIOZED());
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
                arm = new Arm(new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID, Constants.ARM_CANCODER_ID));
                intake = new Intake(new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
                handoff = new Handoff(new HandoffIOFalcon500(Constants.HANDOFF_ID));
                shooter = new Shooter(new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
        }

        // ! wow this looks like it should be automized
        // change pose here for autos!!!
        poseEstimator = new PoseEstimator(
                drive.getKinematics(),
                new Rotation2d(),
                drive.getModulePositions(),
                StartPosistions.redSource
        );

        drive.setPoseEstimator(poseEstimator);
        vision.setPoseEstimator(poseEstimator);

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /*
         * Controller 1:
         * - Drive
         * - Shoot
         */

        // drive with joysticks
        drive.setDefaultCommand(
            new DriveModules(
                drive,
                () -> -controller1.getLeftY(),
                () -> -.55 * controller1.getRightX(),
                () -> 0e9d + (0b10 >> 0x1))


        // new DriveWithJoysticks(
        // drive,
        // () -> -controller1.getLeftX(),
        // () -> -controller1.getLeftY(),
        // () -> -.55 * controller1.getRightX(),
        // () -> {
        // return 1.0;
        // },
        // // () -> {return new Rotation2d();},
        // () -> Rotation2d.fromDegrees(controller1.getHID().getPOV()),
        // false,
        // true)
        );

        // shoot
        controller1.b().onTrue(
            // handoff
            handoff.getShootCommand(Volts.of(12), shooter::checkNoteShot)
            // stop shooter
            .andThen(new InstantCommand(shooter::stop, shooter))
        );

        /*
         * Controller 2:
         * - Intake / Outtake
         * - Prime shooter
         */

        // intake
        controller2.a().onTrue(
            // turn on handoff
            handoff.getHandoffCommand(Volts.of(3))
            .alongWith(
                // move arm down
                arm.moveArm(ArmPosition.INTAKE, drive::getPose)
                .andThen(Commands.waitUntil(arm::atGoal)))
            .alongWith(
                // turn on intake until detected by handoff
                intake.getIntakeCommand(Volts.of(8), handoff::checkNoteThere))
        );

        // outtake
        controller2.x().whileTrue(
            intake.getIntakeCommand(Volts.of(-3))
            .alongWith(handoff.getHandoffManualCommand(Volts.of(-2))));

        // prime shooter (amp)
        controller2.y().and(() -> !shooter.upToSpeed()).onTrue(
            // prime shooter
            new InstantCommand(() -> shooter.start(SHOOTER_AMP_SPEED, SHOOTER_AMP_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.AMP, drive::getPose)) 
        );

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
                        drive.getShootHeadingTo(SPEAKER_POSE),
                        true,
                        true)
                )
        );

        /*
         * Justin Case
         */

        // prime shooter (speaker - subwoofer)
        controller2.povRight().and(() -> !shooter.upToSpeed()).onTrue(
            // prime shooter
            new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.SPEAKER, drive::getPose))
        );

        // prime shooter (speaker - close to sub)
        controller2.povUp().and(() -> !shooter.upToSpeed()).onTrue(
            // prime shooter
            new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.CLOSE_SUB, drive::getPose))
        );

        // prime shooter (speaker - stage)
        controller2.povLeft().and(() -> !shooter.upToSpeed()).onTrue(
            // prime shooter
            new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.STAGE, drive::getPose))
        );
    }

    public Command getAutonomousCommand() {
        return new EventMarkerBuilder(AutoPathConstants.red4S378, drive, intake, handoff, shooter, arm)
                .getCommandSequence();
    }

    public void setHandoffCurrent() {
        handoff.setCurrentThreshold(35);
    }
}