// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SHOOTER_AMP_SPEED;
import static frc.robot.Constants.SHOOTER_LEFT_SPEED;
import static frc.robot.Constants.SHOOTER_RIGHT_SPEED;
import static frc.robot.Constants.SPEAKER_POSE;

import javax.sound.sampled.SourceDataLine;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.*;
import frc.robot.Constants.AutoPathConstants;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.swerveDrive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.utils.EventMarkerBuilder;
import frc.robot.utils.PoseEstimator;
import frc.robot.subsystems.noteIO.arm.*;
import frc.robot.subsystems.noteIO.handoff.*;
import frc.robot.subsystems.noteIO.intake.*;
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
  private final Handoff handoff;
  private final Intake intake;
  private final Shooter shooter;

  private final CommandXboxController controller1 = new CommandXboxController(Constants.CONTROLLER_PORT_1);
  private final CommandXboxController controller2 = new CommandXboxController(Constants.CONTROLLER_PORT_2);

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
            new ModuleIOSparkMax(3));
        camera = new Vision(new CameraIOZED());
        arm = new Arm(
            new ArmIOFalcon500(Constants.ARM_LEAD_ID, Constants.ARM_FOLLOW_ID,
                Constants.ARM_CANCODER_ID));
        intake = new Intake(
            new IntakeIONEO(Constants.INTAKE_ID1, Constants.INTAKE_ID2));
        handoff = new Handoff(new HandoffIOFalcon500(Constants.HANDOFF_ID));
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
        camera = new Vision(new CameraIOZED());
        arm = new Arm(new ArmIOSim());
        handoff = new Handoff(new HandoffIOSim());
        intake = new Intake(new IntakeIOSim()); //////// change
        shooter = new Shooter(new ShooterIOSim());
        break;
      default: // replayed robot
               // disable IO implementations
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOSparkMax(0),
            new ModuleIOSparkMax(1),
            new ModuleIOSparkMax(2),
            new ModuleIOSparkMax(3));
        camera = new Vision(new CameraIOZED());
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

    poseEstimator = new PoseEstimator(
        drive.getKinematics(),
        new Rotation2d(),
        drive.getModulePositions(),
        new Pose2d(
            1.3,
            5.56,
            new Rotation2d()));

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
    // Move drive (future: other subsystems) to track a target
    // ! colin look at this because idk what it's used for
    // Pose3d targetPose = new Pose3d(4, 0, 3, new Rotation3d());

    // Generate a trajectory to a pose when the X button is pressed (and
    // switch
    // drive to position control)
    // String path = AutoPathConstants.THREE_NOTE_WING;
    // new Trigger(() -> {return ((int) Timer.getFPGATimestamp() ==
    // 10);}).onTrue(
    // controller1.x().onTrue(
    // new AutoRoutine(new MechanismsPath(AutoPathConstants.threeNoteWing, drive,
    // intake, shooter, arm))
    // EventMarkerBuilder(AutoPathConstants.threeNoteWingSplits, drive, intake,
    // handoff, shooter, arm).getCommandSequence()
    // new EventMarkerBuilder(AutoPathConstants.fourNoteWingSplits, drive, handoff,
    // shooter,
    // arm).getCommandSequence());

    /*
     * Controller 1:
     * - Driving
     * - Intake / Outtake
     * - Shoot (amp and speaker)
     */

    // drive with joysticks
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> -controller1.getLeftX(),
            () -> -controller1.getLeftY(),
            () -> -.75 * controller1.getRightX(),
            () -> {
              return 1.0;
            },
            () -> Rotation2d.fromDegrees(controller1.getHID().getPOV()),
            false));

    // break when left trigger is held
    // controller1.leftTrigger().whileTrue(
    //     new RunCommand(drive::stopWithX, drive));

    // turn on brake vs coast mode
    // controller1.b().onTrue(
    // Commands.runOnce(drive::toggleDriveMotorsBrakeMode));

    // normal intake (intake stops when it detects note)
    // controller1.a().onTrue(
    // // turn on intake
    // intake.getIntakeCommand(Volts.of(12), intake::checkNoteThere)
    // // turn on handoff
    // .alongWith(handoff.getHandoffCommand(Volts.of(2.7)))
    // // move arm down
    // .alongWith(arm.moveArm(ArmPosition.INTAKE, drive::getPose)));

    // shoot (speaker)
    // controller1.b().and(shooter::upToSpeed).onTrue(
    controller1.b().onTrue(
        // handoff
        handoff.getShootCommand(Volts.of(12), shooter::checkNoteShot)
            // stop shooter
            .andThen(new InstantCommand(shooter::stop, shooter)));

    /*
     * Controller 2:
     * - Prime shooter (amp and speaker)
     */

    // prime shooter (amp)
    controller2.y().and(() -> !shooter.upToSpeed()).onTrue(
        // prime shooter
        new InstantCommand(() -> shooter.start(SHOOTER_AMP_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.AMP, drive::getPose)));

    // prime shooter (speaker)
    controller2.b().and(() -> !shooter.upToSpeed()).onTrue(
        // prime shooter
        new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.SPEAKER, drive::getPose))
            // turn robot towards speaker
            // .alongWith(
            //     new DriveWithJoysticks(
            //         drive,
            //         () -> -controller1.getLeftX(),
            //         () -> -controller1.getLeftY(),
            //         () -> controller1.getRightX(),
            //         () -> {
            //           return 1.0;
            //         },
            //         () -> drive.getPose().getTranslation()
            //             .minus(
            //                 SPEAKER_POSE.getTranslation())
            //             .getAngle()
            //             .plus(new Rotation2d(
            //                 Math.PI)),
            //         true))
    );

    // continuous base intake (intake stops when note is detected in handoff)
    controller2.a().onTrue(
        // turn on handoff
        handoff.getHandoffCommand(Volts.of(6))
             // move arm down
            .alongWith(arm.moveArm(ArmPosition.INTAKE, drive::getPose))
            .alongWith(
                Commands.waitUntil(arm::isAtGoal)
                // turn on intake until detected by handoff
                .andThen(intake.getIntakeCommand(Volts.of(3),
                    handoff::checkNoteThere))
            )
            
    );

    // outtake
    controller2.x().whileTrue(
        new StartEndCommand(
            () -> intake.start(Volts.of(-4)), () -> intake.stop(), intake
        )
        .alongWith(
            new StartEndCommand(
                () -> handoff.start(Volts.of(-6)), () -> handoff.stop(), handoff
            )
        )
    );

    controller2.rightTrigger().onTrue(
        arm.moveArm(ArmPosition.MAIN, drive::getPose)
    );

    controller2.leftTrigger().onTrue(
        new InstantCommand(() -> shooter.stop())
        .alongWith(new InstantCommand(() -> handoff.stop()))
        .alongWith(new InstantCommand(() -> intake.stop()))
    );

    controller2.povUp().onTrue(
        // prime shooter
        new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.SHOOT_FAR, drive::getPose))
    );

    controller2.povDown().onTrue(
        // prime shooter
        new InstantCommand(() -> shooter.start(SHOOTER_LEFT_SPEED, SHOOTER_RIGHT_SPEED), shooter)
            // move arm
            .alongWith(arm.moveArm(ArmPosition.SHOOT_MID, drive::getPose))
    );

    // set arm pos
    // controller1.leftTrigger().onTrue(
    // arm.moveArm(ArmPosition.SHOOT, drive::getPose));
    // controller1.rightTrigger().onTrue(
    // arm.moveArm(ArmPosition.INTAKE, drive::getPose));
    // controller1.leftBumper().onTrue(
    // arm.moveArm(ArmPosition.AMP, drive::getPose));
    // controller1.rightBumper().onTrue(
    // arm.moveArm(ArmPosition.MAIN, drive::getPose));

    // follow nearest apriltag while right trigger is held
    // controller.rightTrigger().whileTrue(new FollowAprilTag(drive, camera));

    // drive to specific pose when right trigger is held
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

    // arm positioning stuff
    // controller.leftTrigger().onTrue(new InstantCommand(() -> {
    // ARM_POSITIONS.put(ArmPosition.RANDY,
    // ARM_POSITIONS.get(ArmPosition.RANDY).minus(Degrees.of(.5)));
    // arm.setArmAngle(ARM_POSITIONS.get(ArmPosition.RANDY));
    // }));
    // controller.rightTrigger().onTrue(new InstantCommand(() -> {
    // ARM_POSITIONS.put(ArmPosition.RANDY,
    // ARM_POSITIONS.get(ArmPosition.RANDY).plus(Degrees.of(.5)));
    // arm.setArmAngle(ARM_POSITIONS.get(ArmPosition.RANDY));
    // }));
    // controller.a().onTrue(new InstantCommand(() -> {
    // System.out.println("RANDY IS : " + ARM_POSITIONS.get(ArmPosition.RANDY));
    // }));

    // third floor test
    // new Trigger(() -> {return ((int) Timer.getFPGATimestamp() == 10);}).onTrue(
    // controller.x().onTrue(
    // drive.runOnce(
    // () -> {
    // String path = "ThirdFloorTest1";
    // var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
    // traj.translateBy(traj.positionTrajectory.get(0).getTranslation().unaryMinus());

    // record robot trajectory
    // System.out.println("recording pos traj");
    // Logger.recordOutput("Auto/GeneratedTrajectory",
    // traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));
    // System.out.println("Writing trajectory to CSV");
    // traj.toCSV(path);
    // drive.runPosition(traj);

    // I have no idea what this code is supposed to do
    // var traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
    // adjust so that the start of the trajectory is where the robot is
    // traj.translateBy(traj.positionTrajectory.get(0).getTranslation().unaryMinus());
    // traj.translateBy(drive.getPose().getTranslation());
    // System.out.println("recording pos traj");
    // Logger.recordOutput("Auto/GeneratedTrajectory",
    // traj.positionTrajectory.toArray(new Pose2d[traj.positionTrajectory.size()]));
    // System.out.println("Writing trajectory to CSV");
    // traj.toCSV(path);
    // drive.runPosition(traj);
    // }).asProxy().andThen(new AutoRoutine(drive,new MechanismsPath(path,intake)))

    // sysID
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