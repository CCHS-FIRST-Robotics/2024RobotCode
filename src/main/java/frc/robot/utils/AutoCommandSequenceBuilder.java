package frc.robot.utils;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.Pair;
import com.choreo.lib.*;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.noteIO.arm.Arm;
import frc.robot.subsystems.noteIO.handoff.Handoff;
import frc.robot.subsystems.noteIO.intake.Intake;
import frc.robot.subsystems.noteIO.shooter.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.*;

public final class AutoCommandSequenceBuilder {
    private Drive drive;
    private Intake intake;
    private Handoff handoff;
    private Shooter shooter;
    private Arm arm;

    private Command autoCommandSequence;

    public AutoCommandSequenceBuilder(
        ArrayList<String> pathList, 
        Drive drive, 
        Arm arm,
        Intake intake, 
        Handoff handoff,
        Shooter shooter
    ){
        this.drive = drive;
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        this.handoff = handoff;

        // append one AutoCommand per path section to the sequence
        for (String path : pathList) {
            addCommand(path);
        }

        // ! I think that this is here just in case
        autoCommandSequence = autoCommandSequence.andThen(
            new InstantCommand(() -> shooter.stop())
            .alongWith(new InstantCommand(() -> handoff.stop()))
        );
    }

    public void addCommand(String path) {
        List<Pair<Double, Command>> events = new ArrayList<Pair<Double, Command>>();

        // the time it takes to perform each item
        double driveTime = Math.max(AutoConstants.MAX_ARM_MOVE_TIME, Choreo.getTrajectory(path).getTotalTime());
        double intakeTime = AutoConstants.INTAKE_TIME;
        double moveArmTime = AutoConstants.MAX_ARM_MOVE_TIME;
        double shootTime =  AutoConstants.SHOOT_TIME;
        double totalTime = driveTime + intakeTime + moveArmTime + shootTime;

        // drive to the next waypoint
        // move arm to intake
        // turn on intake and handoff
        // prime shooter
        events.add(
            Pair.of(
                0.0,
                drive.followTrajectory(DriveTrajectoryGenerator.generateChoreoTrajectory(path))
                .alongWith(arm.moveArm(ArmPosition.INTAKE, drive::getPose))
                .alongWith(intake.getIntakeCommand(AutoConstants.INTAKE_VOLTS, handoff::checkNoteThere))
                .alongWith(handoff.getHandoffCommand(AutoConstants.HANDOFF_IN_VOLTS))
                .alongWith(new InstantCommand(() -> shooter.start(Constants.SHOOT_SPEED_LEFT, Constants.SHOOT_SPEED_RIGHT), shooter))
            )
        );

        // move arm to shoot
        events.add(
            Pair.of(
                driveTime + intakeTime, 
                arm.moveArm(ArmPosition.SHOOT, drive::getPose)
            )
        );
        
        // run handoff to shoot
        events.add(
            Pair.of(
                driveTime + intakeTime + moveArmTime, 
                handoff.getShootCommand(AutoConstants.HANDOFF_OUT_VOLTS, shooter::checkNoteShot)
            )
        );

        // append to the command sequence
        autoCommandSequence = autoCommandSequence == null ? 
            (new AutoCommand(events, totalTime)) 
            : autoCommandSequence.andThen(new AutoCommand(events, totalTime));
    }

    public Command getAutoCommandSequence() {
        return autoCommandSequence;
    }
}