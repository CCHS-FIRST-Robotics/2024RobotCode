// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Constants.StartPosistions;

public class RobotContainer {
    private final Drive drive;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Vision vision;

    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT);

    public RobotContainer() {
        drive = new Drive(
            new GyroIO(){}, // ! what is this
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim()
        );
        vision = new Vision(new CameraIOZED(), new CameraIOPhotonVision());

        poseEstimator = new SwerveDrivePoseEstimator(
                drive.getKinematics(),
                new Rotation2d(),
                drive.getModulePositions(),
                StartPosistions.redSource,  // ! wow this looks like it should be automized
                Constants.defaultStateStdDevs,
                Constants.defaultPVMeasurementStdDevs
        );

        drive.setPoseEstimator(poseEstimator);
        vision.setPoseEstimator(poseEstimator);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                () -> -controller.getLeftX(),
                () -> -controller.getLeftY(),
                () -> -.55 * controller.getRightX(),
                () -> {
                    return 1.0;
                },
                () -> Rotation2d.fromDegrees(controller.getHID().getPOV()), // ! the hell is this
                false,
                true
            )
        );
    }
}