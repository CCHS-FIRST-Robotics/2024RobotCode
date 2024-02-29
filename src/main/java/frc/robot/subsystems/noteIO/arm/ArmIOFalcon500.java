package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmIOFalcon500 implements ArmIO {
    /* MOTOR CONTROLLERS + PID */
    private final TalonFX driveFalcon;
    private final TalonFXConfiguration driveFalconConfig = new TalonFXConfiguration();

    // Uhh Feedforward momment!
    private static final double driveFeedforwardKg = 0;
    // private static final double driveFeedforwardKs = 0;
    // private static final double driveFeedforwardKv = 0;
    // private static final double driveFeedforwardKa = 0;

    private CANcoder driveCancoder;

    // TODO - USE FOC (Field Oriented Control) - MotionMagicTorqueCurrentFOC
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
    private static final double driveKp = 0;
    private static final double driveKd = 0.0d;
    private static final double driveKi = 0.0d;
    private static final double driveKv = 0.113; // (from falcon500 spec sheet) UNITS: Volts / (Rotations / Second)

    private final double gearRatio = 100.0;

    // private final boolean motorInverted = false;
    private final Rotation2d absoluteEncoderOffset = new Rotation2d();

    int index;

    public ArmIOFalcon500(int motorID, int cancoderID) {
        driveFalcon = new TalonFX(motorID);
        /// I encode???? Turst Different ids needed NO IDEA WHAT IDS
        driveCancoder = new CANcoder(cancoderID);

        drivePositionSignal = driveFalcon.getPosition();
        driveVelocitySignal = driveFalcon.getVelocity();
        driveAppliedVoltageSignal = driveFalcon.getMotorVoltage();
        driveCurrentSignal = driveFalcon.getSupplyCurrent();
        driveTempSignal = driveFalcon.getDeviceTemp();

        absolutePositionSignal = driveCancoder.getPosition(); // check what this does!!!!!!! absolute variations! other
                                                              // methord!!!!
        absoluteVelocitySignal = driveCancoder.getVelocity();

        rotorPositionSignal = driveFalcon.getRotorPosition();
        faultFusedSensorOutOfSync = driveFalcon.getFault_FusedSensorOutOfSync();
        stickyFaultFusedSensorOutOfSync = driveFalcon.getStickyFault_FusedSensorOutOfSync();
        faultRemoteSensorOutOfSync = driveFalcon.getFault_RemoteSensorDataInvalid();
        stickyFaultRemoteSensorOutOfSync = driveFalcon.getStickyFault_RemoteSensorDataInvalid();

        driveMMConfig.MotionMagicCruiseVelocity = 5; // 1 rotation every 1 seconds
        driveMMConfig.MotionMagicAcceleration = 10; // 1 second to reach max speed
        driveMMConfig.MotionMagicJerk = 30; // .1 seconds to reach max accel

        // Feedforward momment!

        // Ya-yoink! (from
        // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
        // zeros the magnet!
        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
        CANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset = 0.4; // CHANGGE
        driveCancoder.getConfigurator().apply(CANcoderConfig);

        // fuses (trust)
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

        talonFXConfig.Feedback.FeedbackRemoteSensorID = driveCancoder.getDeviceID();
        talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonFXConfig.Feedback.SensorToMechanismRatio = 1.0; // fuck maybe? CHANGE????
        talonFXConfig.Feedback.RotorToSensorRatio = 100.0; // CHNAGE

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = 60;

        driveFalcon.getConfigurator().apply(talonFXConfig);

        drivePID.kP = driveKp;
        drivePID.kI = driveKi;
        drivePID.kD = driveKd;
        drivePID.kV = driveKv;
        drivePID.GravityType = GravityTypeValue.Arm_Cosine;
        // drivePID.kA = driveFeedforwardKa; //dont use it (forn now)(trust) (use it
        // ocne sysid works)
        drivePID.kG = driveFeedforwardKg;
        // drivePID.kS = driveFeedforwardKs;
        // drivePID.kV = driveFeedforwardKv; // max rpm is 6,380 volts * seconds /
        // radians
        // Units needed are volts * seconds / radians
        drivePID.kV = 18d / (Math.PI * 319d); // TRUST!!!!! I don't!!!
        // 6380 rotaions per minute is 319/3 rotations per second
        // 2pi * 319/3 radians per second <--- you don't need to convert to radians
        // 3/(2pi * 319) seconds per radian
        // 12 * 3/(2pi * 319)

        driveFeedbackConfig.SensorToMechanismRatio = gearRatio;

        driveFalconConfig.Voltage.PeakForwardVoltage = 12;
        driveFalconConfig.Voltage.PeakReverseVoltage = -12;

        driveFalcon.setPosition(absoluteEncoderOffset.getRotations());

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
                stickyFaultRemoteSensorOutOfSync);

        inputs.drivePosition = Rotations.of(drivePositionSignal.getValueAsDouble());
        inputs.driveVelocity = RotationsPerSecond.of(driveVelocitySignal.getValueAsDouble());
        inputs.driveAppliedVolts = Volts.of(driveAppliedVoltageSignal.getValueAsDouble());
        inputs.driveCurrent = Amps.of(driveCurrentSignal.getValueAsDouble());
        inputs.driveTemp = Celsius.of(driveTempSignal.getValueAsDouble());
        inputs.absoluteArmPosition = Rotations.of(absolutePositionSignal.getValueAsDouble());

        // BAD - RotationsPerSecond is for UNITS, it won't convert a position into a
        // velocity
        // inputs.absoluteArmVelocity =
        // RotationsPerSecond.of(absolutePositionSignal.getValueAsDouble());
        inputs.absoluteArmVelocity = RotationsPerSecond.of(absoluteVelocitySignal.getValueAsDouble());

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
    public void setDrivePosition(Measure<Angle> position) {
        // dont let the arm go out of the rage [-30, 270]
        position = Radians.of(MathUtil.clamp(position.in(Radians), -Math.PI / 6.0, 4 * Math.PI / 3.0));

        // lol this is NOT going to work
        driveFalcon.setControl(driveMotionMagic.withPosition(position.in(Rotations)).withSlot(0));
    }
}