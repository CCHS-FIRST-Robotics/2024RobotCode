package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmIOFalcon500 implements ArmIO {
    /* MOTOR CONTROLLERS + PID */
    private final TalonFX driveFalcon;
    private final TalonFXConfiguration driveFalconConfig = new TalonFXConfiguration();

    // Uhh Feedforward momment!
    private static final double driveFeedforwardKg = 0;
    private static final double driveFeedforwardKs = 0;
    private static final double driveFeedforwardKv = 0;
    private static final double driveFeedforwardKa = 0;

    // I rise! FeedFwd momment!
    // Jk i fall!

    // Can code? Cancoder
    private CANcoder driveCancoder;


    private final MotionMagicVoltage driveMotionMagic = new MotionMagicVoltage(0);
    private final MotionMagicConfigs driveMMConfig = driveFalconConfig.MotionMagic;
    private final Slot0Configs drivePID = driveFalconConfig.Slot0;
    private final FeedbackConfigs driveFeedbackConfig = driveFalconConfig.Feedback;

    StatusSignal<Double> drivePositionSignal;
    StatusSignal<Double> driveVelocitySignal;
    StatusSignal<Double> driveAppliedVoltageSignal;
    StatusSignal<Double> driveCurrentSignal;
    StatusSignal<Double> driveTempSignal;

    StatusSignal<Double> absolutePositionSignal;
    StatusSignal<Double> absoluteVelocitySignal;

    StatusSignal<Double> rotorPositionSignal;

    StatusSignal<Boolean> faultFusedSensorOutOfSync;
    StatusSignal<Boolean> stickyFaultFusedSensorOutOfSync;
    StatusSignal<Boolean> faultRemoteSensorOutOfSync;
    StatusSignal<Boolean> stickyFaultRemoteSensorOutOfSync;
    


    // TODO: update constants in periodic once tunable is set up
    private static final double driveKp = 100;
    private static final double driveKd = 0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000d;
    private static final double driveKi = 0.000000d;
    private static final double driveKv = 0.113; // (from falcon500 spec sheet) UNITS: Volts / (Rotations / Second)

    private final double gearRatio = 100.0;

    private final boolean isMotorInverted = false;
    private final Rotation2d absoluteEncoderOffset = new Rotation2d();

    Orchestra orchestra = new Orchestra();

    int index;

    public ArmIOFalcon500(int motorID, int cancoderID) {
        driveFalcon = new TalonFX(motorID);
        drivePositionSignal = driveFalcon.getPosition();
        driveVelocitySignal = driveFalcon.getVelocity();
        driveAppliedVoltageSignal = driveFalcon.getMotorVoltage();
        driveCurrentSignal = driveFalcon.getSupplyCurrent();
        driveTempSignal = driveFalcon.getDeviceTemp();

    
        absolutePositionSignal = driveCancoder.getPosition(); // check what this does!!!!!!! absolute variations! other methord!!!!
        absoluteVelocitySignal = driveCancoder.getVelocity();

        rotorPositionSignal = driveFalcon.getRotorPosition();
        faultFusedSensorOutOfSync = driveFalcon.getFault_FusedSensorOutOfSync();
        stickyFaultFusedSensorOutOfSync = driveFalcon.getStickyFault_FusedSensorOutOfSync();
        faultRemoteSensorOutOfSync = driveFalcon.getFault_RemoteSensorDataInvalid();
        stickyFaultFusedSensorOutOfSync = driveFalcon.getStickyFault_RemoteSensorDataInvalid();


        driveMMConfig.MotionMagicCruiseVelocity = 5; // 1 rotation every 1 seconds
        driveMMConfig.MotionMagicAcceleration = 10; // 1 second to reach max speed
        driveMMConfig.MotionMagicJerk = 30; // .1 seconds to reach max accel
        
        /// I encode???? Turst Different ids needed NO IDEA WHAT IDS
        driveCancoder = new CANcoder(cancoderID);
        
        // Feedforward momment!

        // Ya-yoink! (from  https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
        // zeros the magnet!
        CANcoderConfiguration canCodeConfig = new CANcoderConfiguration();
        canCodeConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCodeConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCodeConfig.MagnetSensor.MagnetOffset = 0.4; // CHANGGE
        driveCancoder.getConfigurator().apply(canCodeConfig);

        // fuses (trust)
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.Feedback.FeedbackRemoteSensorID = driveCancoder.getDeviceID();
        talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfig.Feedback.SensorToMechanismRatio = 1.0; // fuck maybe? CHANGE????
        talonFXConfig.Feedback.RotorToSensorRatio = 100.0; // CHNAGE

        driveFalcon.getConfigurator().apply(talonFXConfig);


        drivePID.kP = driveKp;
        drivePID.kI = driveKi;
        drivePID.kD = driveKd;
        drivePID.kV = driveKv; 
        drivePID.GravityType = GravityTypeValue.Arm_Cosine;
        // drivePID.kA = driveFeedforwardKa; //dont use it (forn now)(trust) (use it ocne ssysid works)
        drivePID.kG = driveFeedforwardKg;
        // drivePID.kS = driveFeedforwardKs;
        // drivePID.kV = driveFeedforwardKv; // max rpm is 6,380  volts * seconds / radians
        // Units needed are volts * seconds / radians
        drivePID.kV = 18d / (Math.PI * 319d); // TRUST!!!!! I don't!!!
        // 6380 rotaions per minute is 319/3 rotations per second
        // 2pi * 319/3 radians per second
        // 3/(2pi * 319) seconds per radian
        // 12 * 3/(2pi * 319)

        driveFeedbackConfig.SensorToMechanismRatio = gearRatio;

        driveFalconConfig.Voltage.PeakForwardVoltage = 12;
        driveFalconConfig.Voltage.PeakReverseVoltage = -12;

        driveFalcon.setPosition(absoluteEncoderOffset.getRotations());

        orchestra.addInstrument(driveFalcon);

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = driveFalcon.getConfigurator().apply(driveFalconConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        // tatus!!!!

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePositionSignal,
            driveVelocitySignal,
            driveAppliedVoltageSignal,
            driveCurrentSignal,
            driveTempSignal,
            absolutePositionSignal,
            absoluteVelocitySignal,
            rotorPositionSignal,
            faultFusedSensorOutOfSync,
            stickyFaultFusedSensorOutOfSync,
            faultRemoteSensorOutOfSync,
            stickyFaultRemoteSensorOutOfSync
        );


        inputs.drivePosition = Rotations.of(drivePositionSignal.getValueAsDouble());
        inputs.driveVelocity = RotationsPerSecond.of(driveVelocitySignal.getValueAsDouble());
        inputs.driveAppliedVolts = Volts.of(driveAppliedVoltageSignal.getValueAsDouble());
        inputs.driveCurrent = Amps.of(driveCurrentSignal.getValueAsDouble());
        inputs.driveTemp = Celsius.of(driveTempSignal.getValueAsDouble());
        inputs.absoluteArmPosition = Rotations.of(absolutePositionSignal.getValueAsDouble());
        inputs.absoluteArmVelocity = RotationsPerSecond.of(absolutePositionSignal.getValueAsDouble());
        inputs.rotorPositionSignal = Rotations.of(rotorPositionSignal.getValueAsDouble());
        // CHeck with colin if it works
        inputs.faultFusedSensorOutOfSync = faultFusedSensorOutOfSync.getValue();
        inputs.stickyFaultFusedSensorOutOfSync = stickyFaultFusedSensorOutOfSync.getValue();
        inputs.faultRemoteSensorOutOfSync = faultRemoteSensorOutOfSync.getValue();
        inputs.stickyFaultRemoteSensorOutOfSync = stickyFaultRemoteSensorOutOfSync.getValue();
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        driveFalcon.setVoltage(volts.in(Volts));
    }

    @Override
    public void setDrivePosition(Measure<Angle> positionRad) {
        // for testing, dont let the arm go past 90 degrees in either direction
        // positionRad = Radians.of(MathUtil.clamp(positionRad.in(Radians), -Math.PI/2.0, Math.PI/2.0));
        positionRad = Radians.of(MathUtil.clamp(positionRad.in(Radians), -Math.PI/6.0, 4 * Math.PI/3.0));  //uhh probably incorrect fix?
        // driveFalcon.setControl(driveMotionMagic.withPosition(positionRad.in(Rotations)).withSlot(0));

        // lol this is NOT going to work
        driveFalcon.setControl(driveMotionMagic.withPosition(positionRad.in(Rotations)).withSlot(0));

        // use cancodee ~ JK DO NOT USE CANCODE
        // driveEncoder.setControl(driveMotionMagic.withPosition(positionRad.in(Rotations)).withSlot(0));



        
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
    // driveFalcon.setBrakeMode(enable);
    // }
}