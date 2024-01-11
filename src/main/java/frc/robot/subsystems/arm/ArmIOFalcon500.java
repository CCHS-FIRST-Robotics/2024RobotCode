package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

public class ArmIOFalcon500 implements ArmIO {
    /* MOTOR CONTROLLERS + PID */
    private final TalonFX driveFalcon;
    private final TalonFXConfiguration driveFalconConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage driveMotionMagic = new MotionMagicVoltage(0);
    private final MotionMagicConfigs driveMMConfig = driveFalconConfig.MotionMagic;
    private final Slot0Configs drivePID = driveFalconConfig.Slot0;
    private final FeedbackConfigs driveFeedbackConfig = driveFalconConfig.Feedback;

    StatusSignal<Double> drivePositionSignal;
    StatusSignal<Double> driveVelocitySignal;
    StatusSignal<Double> driveAppliedVoltageSignal;
    StatusSignal<Double> driveCurrentSignal;
    StatusSignal<Double> driveTempSignal;

    // TODO: update constants in periodic once tunable is set up
    private static final double driveKp = 100;
    private static final double driveKd = 0.0;
    private static final double driveKi = 0.000000;
    private static final double driveKv = 0.113; // (from falcon500 spec sheet) UNITS: Volts / (Rotations / Second)

    private final double gearRatio = 100.0;

    private final boolean isMotorInverted = false;
    private final Rotation2d absoluteEncoderOffset = new Rotation2d();

    Orchestra orchestra = new Orchestra();

    int index;

    public ArmIOFalcon500(int motorID) {
        driveFalcon = new TalonFX(motorID);
        drivePositionSignal = driveFalcon.getPosition();
        driveVelocitySignal = driveFalcon.getVelocity();
        driveAppliedVoltageSignal = driveFalcon.getMotorVoltage();
        driveCurrentSignal = driveFalcon.getSupplyCurrent();
        driveTempSignal = driveFalcon.getDeviceTemp();

        driveMMConfig.MotionMagicCruiseVelocity = 5; // 1 rotation every 1 seconds
        driveMMConfig.MotionMagicAcceleration = 10; // 1 second to reach max speed
        driveMMConfig.MotionMagicJerk = 30; // .1 seconds to reach max accel

        drivePID.kP = driveKp;
        drivePID.kI = driveKi;
        drivePID.kD = driveKd;
        drivePID.kV = driveKv;

        driveFeedbackConfig.SensorToMechanismRatio = gearRatio;

        driveFalconConfig.Voltage.PeakForwardVoltage = 12;
        driveFalconConfig.Voltage.PeakReverseVoltage = -12;

        driveFalcon.setPosition(absoluteEncoderOffset.getRotations());

        orchestra.addInstrument(driveFalcon);

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            status = driveFalcon.getConfigurator().apply(driveFalconConfig);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePositionSignal,
            driveVelocitySignal,
            driveAppliedVoltageSignal,
            driveCurrentSignal,
            driveTempSignal
        );

        inputs.drivePosition = Rotations.of(drivePositionSignal.getValueAsDouble());
        inputs.driveVelocity = RotationsPerSecond.of(driveVelocitySignal.getValueAsDouble()) ;
        inputs.driveAppliedVolts = Volts.of(driveAppliedVoltageSignal.getValueAsDouble());
        inputs.driveCurrent = Amps.of(driveCurrentSignal.getValueAsDouble());
        inputs.driveTemp = Celsius.of(driveTempSignal.getValueAsDouble());
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        driveFalcon.setVoltage(volts.in(Volts));
    }

    @Override
    public void setDrivePosition(Measure<Angle> positionRad) {
        driveFalcon.setControl(driveMotionMagic.withPosition(positionRad.in(Rotations)).withSlot(0));
    }

    public void setMusicTrack(String path) {
        // Attempt to load the chrp
        var status = orchestra.loadMusic(path);

        if (!status.isOK()) {
        // log error
            System.out.println("MUSIC LOADING ERROR\n" + status.toString());
        }
    }

    public void playMusic() {
        orchestra.play();
    }

    public void pauseMusic() {
        orchestra.pause();
    }

    public void stopMusic() {
        orchestra.stop();
    }

    // @Override
    // public void setDriveBrakeMode(boolean enable) {
    //     driveFalcon.setBrakeMode(enable);
    // }
}
