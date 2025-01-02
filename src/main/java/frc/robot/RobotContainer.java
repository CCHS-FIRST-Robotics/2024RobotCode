// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.commands.*;
import frc.robot.utils.PoseEstimator;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.noteIO.arm.*;
import frc.robot.subsystems.noteIO.handoff.*;
import frc.robot.subsystems.noteIO.intake.*;
import frc.robot.subsystems.noteIO.shooter.*;
import frc.robot.utils.AutoCommandSequenceBuilder;
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

    // ! yeah do something with this
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSparkMax(0),
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3)
                );
                vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());
                arm = new Arm(new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID, Constants.ARM_CANCODER_ID));
                intake = new Intake(new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
                handoff = new Handoff(new HandoffIOSparkMax(Constants.HANDOFF_ID));
                shooter = new Shooter(new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
            case SIM:
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim()
                );
                vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());
                arm = new Arm(new ArmIOSim());
                handoff = new Handoff(new HandoffIOSim());
                intake = new Intake(new IntakeIOSim());
                shooter = new Shooter(new ShooterIOSim());
                break;
            default:
                drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSparkMax(0),
                    new ModuleIOSparkMax(1),
                    new ModuleIOSparkMax(2),
                    new ModuleIOSparkMax(3)
                );
                vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());
                arm = new Arm(new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID, Constants.ARM_CANCODER_ID));
                intake = new Intake(new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
                handoff = new Handoff(new HandoffIOSparkMax(Constants.HANDOFF_ID));
                shooter = new Shooter(new ShooterIOFalcon500(Constants.SHOOTER_ID_1, Constants.SHOOTER_ID_2));
                break;
        }

        // ! wow this looks like it should be automized
        // change pose here for autos!!!
        poseEstimator = new PoseEstimator(
            drive.getKinematics(),
            new Rotation2d(),
            drive.getModulePositions(),
            new Pose2d(0, 0, new Rotation2d())
        );

        drive.setPoseEstimator(poseEstimator);
        vision.setPoseEstimator(poseEstimator);

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                () -> -0.3 * controller1.getLeftX(), //! get rid of these magic numbers or at least move them
                () -> -0.3 * controller1.getLeftY(), 
                () -> -0.7 * controller1.getRightX()
            )
        );
    }

    public Command getAutonomousCommand() {
        return new AutoCommandSequenceBuilder(
            AutoPathConstants.twoStraight, // specifies the auto to run
            drive, 
            arm, 
            intake, 
            handoff, 
            shooter
        ).getAutoCommandSequence();
    }
}