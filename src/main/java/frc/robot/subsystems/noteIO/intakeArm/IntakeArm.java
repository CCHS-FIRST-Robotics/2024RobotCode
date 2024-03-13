package frc.robot.subsystems.noteIO.intakeArm;

import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants.AutoPathConstants;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeArm extends SubsystemBase {
    private IntakeArmIO io;
    private Measure<Voltage> volts = Volts.of(0);
    private double startTime;
    // Debouncer currentDebouncer = new Debouncer(0.3,
    // Debouncer.DebounceType.kRising);
    private IntakeArmIOInputsAutoLogged inputs = new IntakeArmIOInputsAutoLogged();
    private boolean intake = false;
    private boolean shoot = false;

    public IntakeArm(IntakeArmIO io) {
        this.io = io;
    }

    public void start(Measure<Voltage> v) {
        volts = v;
        startTime = Timer.getFPGATimestamp();
    }

    public void stop() {
        volts = Volts.of(0);
    }

    @AutoLogOutput
    private boolean checkNoteThere() {
        return inputs.motorCurrent > (25d / 3d) * volts.in(Volts) && (Timer.getFPGATimestamp() - startTime > 0.5);
        // return Timer.getFPGATimestamp() - startTime >
        // AutoPathConstants.Q_INTAKE_TIME;
        // return currentDebouncer.calculate(inputs.motorCurrent > 30);
        // return inputs.motorCurrent > 30 && inputs.motorVelocity > 98 * (volts / 12);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("handoff", inputs);
        Logger.recordOutput("Handoff On", volts.magnitude() != 0);
        Logger.recordOutput("Q Shoot", shoot);
        Logger.recordOutput("Q Intake", intake);
        io.setVoltage(volts);
    }

    public Command getHandoffCommand(Measure<Voltage> v) {
        // turns motor on until note detected
        return new FunctionalCommand(
                () -> start(v),
                () -> {
                    intake = true;
                },
                (interrupted) -> {
                    intake = false;
                    stop();
                },
                () -> checkNoteThere(),
                this);
    }

    // turns motor on until shooter detects note
    public Command getShootCommand(Measure<Voltage> v, BooleanSupplier shooterDone) {
        // turns motor on until shooter detects note

        return new FunctionalCommand(
                () -> start(v),
                () -> {
                    shoot = true;
                },
                (interrupted) -> {
                    shoot = false;
                    stop();
                },
                shooterDone,
                this);
    }
}