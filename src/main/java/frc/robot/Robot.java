// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.urcl.URCL;
import com.ctre.phoenix6.SignalLogger;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    Command autonomousCommand;

    @Override
    public void robotInit() {
        // record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // set up data recievers
        // switch (Constants.CURRENT_MODE) {
        //     case REAL: // log to a USB stick
        //         Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        //         Logger.addDataReceiver(new NT4Publisher());
        //         // new PowerDistribution(0, ModuleType.kCTRE); // enables power distribution
        //         break;
        //     case SIM: // log to local folder
        //         Logger.addDataReceiver(new WPILOGWriter());
        //         Logger.addDataReceiver(new NT4Publisher());
        //         break;
        //     case REPLAY: // set up replay source
        //         setUseTiming(false); // run as fast as possible
        //         String logPath = LogFileUtil.findReplayLog();
        //         Logger.setReplaySource(new WPILOGReader(logPath));
        //         Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        //         break;
        // }

        SignalLogger.setPath("/media/sda1/");
        SignalLogger.start();

        Logger.registerURCL(URCL.startExternal());
        Logger.start();

        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.setHandoffCurrent();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    // @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}