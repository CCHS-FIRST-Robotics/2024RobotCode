package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.*;
import com.choreo.lib.*;
import edu.wpi.first.math.Pair;
import java.util.*;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.handoff.Handoff;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import frc.robot.Constants.*;

public final class EventMarkerBuilder {
    private Drive drive;
    private Intake intake;
    private Handoff handoff;
    private Shooter shooter;
    private Arm arm;

    private Command command;

    ArrayList<Double> timestamps;

    public EventMarkerBuilder(ArrayList<String> pathList, Drive drive, Intake intake, Handoff handoff,
            Shooter shooter, Arm arm) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.arm = arm;
        this.handoff = handoff;

        command = Commands.waitSeconds(1).andThen(new InstantCommand(
                () -> shooter.start(AutoPathConstants.SHOOT_SPEED_LEFT, AutoPathConstants.SHOOT_SPEED_RIGHT), shooter)
                .andThen(
                        arm.moveArm(ArmPosition.SPEAKER, drive::getPose)
                                .alongWith(
                                        Commands.waitUntil(() -> shooter.upToSpeed() && arm.atGoal())
                                                .andThen(handoff.getShootCommand(AutoPathConstants.HANDOFF_IN_VOLTS,
                                                        shooter::checkNoteShot)))
                                .until(shooter::checkNoteShot)));

        for (String path : pathList) {
            addCommand(path);
        }

        System.out.println("fihffhusdhfd");

        command = command.andThen(
                new InstantCommand(() -> shooter.stop())
                        .alongWith(new InstantCommand(() -> handoff.stop())));

    }

    public void addCommand(String path) {
        DriveTrajectory traj = DriveTrajectoryGenerator.generateChoreoTrajectoryFromFile(path);
        ChoreoTrajectory choreoTrajectory = Choreo.getTrajectory(path); // only being used rn to get total time
        List<Pair<Double, Command>> eventMarkers = new ArrayList<Pair<Double, Command>>();

        double driveTime = Math.max(AutoPathConstants.MAX_ARM_MOVE_TIME, choreoTrajectory.getTotalTime());
        double intakeTime = driveTime + AutoPathConstants.INTAKE_TIME;
        double shootTime = intakeTime + AutoPathConstants.MAX_ARM_MOVE_TIME;
        double totalTime = shootTime + AutoPathConstants.SHOOT_TIME;

        // start drive and more arm to inake
        eventMarkers.add(Pair.of(AutoPathConstants.INIT_MOVEMENTS_TIME,
                // turn on handoff
                handoff.getHandoffCommand(AutoPathConstants.HANDOFF_IN_VOLTS)
                        // turn on intake (no matter where arm is)
                        .alongWith(intake.getIntakeCommand(AutoPathConstants.INTAKE_VOLTS, handoff::checkNoteThere))
                        // move arm down
                        .alongWith(arm.moveArm(ArmPosition.INTAKE, drive::getPose))
                        // drive
                        .alongWith(drive.followTrajectory(traj))
        // turn on intake (when arm is down)
        // .alongWith(
        // Commands.waitUntil(arm::isAtGoal)
        // // turn on intake until detected by handoff
        // .andThen(intake.getIntakeCommand(AutoPathConstants.INTAKE_VOLTS,
        // handoff::checkNoteThere))
        // ))
        ));

        // arm shoot
        eventMarkers.add(Pair.of(intakeTime,
                arm.moveArm(ArmPosition.SHOOT, drive::getPose)));

        // run handoff & shoot
        eventMarkers.add(Pair.of(shootTime,
                handoff.getShootCommand(AutoPathConstants.HANDOFF_OUT_VOLTS,
                        shooter::checkNoteShot)));

        if (command == null) {
            command = (new AutoCommand(eventMarkers, totalTime));
        } else {
            command = command.andThen((new AutoCommand(eventMarkers, totalTime)));
        }
    }

    // public static ArrayList<Double> getEventMarkers(String path) {
    // var traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
    // var traj_file = new File(traj_dir, path + ".traj");

    // var timestamps = new ArrayList<Double>();

    // try {
    // // parsing file "JSONExample.json"
    // Object obj = new JSONParser().parse(new FileReader(traj_file));
    // // typecasting obj to JSONObject
    // JSONObject jo = (JSONObject) obj;

    // // getting timestamps
    // JSONArray eventMarkers = (JSONArray) jo.get("eventMarkers");
    // eventMarkers.forEach((marker) -> {
    // timestamps.add(
    // (Double) ((JSONObject) marker).get("timestamp")
    // );
    // });

    // } catch (Exception ex) {
    // DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
    // }

    // return timestamps;
    // }

    public Command getCommandSequence() {
        return command;
    }
}
