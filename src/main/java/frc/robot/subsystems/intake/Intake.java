package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import frc.robot.Constants;

// switch statements or smth
// too many set voltage methods

// connects RobotContainer and the command scheduler with the motor object
public class Intake extends SubsystemBase {
    IntakeIOCIM io;
    boolean buttonHeld;
    boolean noteThere;
    SlewRateLimiter voltLimiter = new SlewRateLimiter(6);
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged(); // class created by MotorIO interface

    public Intake() {
        io = new IntakeIOCIM(12, 13);
    }

    public void start(int volts) {
        io.setVoltage(voltLimiter.calculate(volts));
    }

    public void stop() {
        buttonHeld = false;
    }

    @Override
    public void periodic() {
        noteThere = io.motor.getSupplyCurrent() > 12;

        if (!buttonHeld && !noteThere) {
            io.setVoltage(0);
            voltLimiter.reset(0);
        }

        // log data
        io.updateInputs(inputs);
        Logger.processInputs("motorVoltage", inputs);
        Logger.recordOutput("motorCurrent", io.motor.getSupplyCurrent());
    }

    public Command startEndCommmand() {
        return startEnd(() -> start(12), () -> stop());
    }
}