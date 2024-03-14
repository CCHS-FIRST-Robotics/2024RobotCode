package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
// import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;

public class ArmIOFalcon500 implements ArmIO {
    /* MOTOR CONTROLLERS + PID */
    private final TalonFX driveFalcon;
    private final TalonFXConfiguration driveFalconConfig = new TalonFXConfiguration();
    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

    private CANcoder driveCancoder;

    // TODO - USE FOC (Field Oriented Control) - MotionMagicTorqueCurrentFOC
    private final MotionMagicVoltage driveMotionMagic = new MotionMagicVoltage(0);
    // private final MotionMagicTorqueCurrentFOC driveMotionMagic = new
    // MotionMagicTorqueCurrentFOC(0);
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

    StatusSignal<Double> closedLoopReferenceSignal;
    StatusSignal<Double> closedLoopErrorSignal;

    StatusSignal<Boolean> faultFusedSensorOutOfSync;
    StatusSignal<Boolean> stickyFaultFusedSensorOutOfSync;
    StatusSignal<Boolean> faultRemoteSensorOutOfSync;
    StatusSignal<Boolean> stickyFaultRemoteSensorOutOfSync;

    private static final double gearRatio = 100 * 54 / 15d;

    // TODO: update constants in periodic once tunable is set up
    private static final double driveKp = 150;
    private static final double driveKd = 0d;
    private static final double driveKi = 0.0d;

    // Uhh Feedforward momment!
    private static final double driveFeedforwardKg = 0.435; // .435V
    private static final double driveFeedforwardKs = 0;
    // Units needed are volts * seconds / rotations, max rpm is 6,380
    private static final double driveFeedforwardKv = 12 * (3 / 319d) / gearRatio; // 6380 rotaions per minute is 319/3
                                                                                  // rotations per second
    private static final double driveFeedforwardKa = 0;

    // private final boolean motorInverted = false;
    private final Measure<Angle> absoluteEncoderOffset = Radians.of(-3.71);

    int index;

    public ArmIOFalcon500(int motorID, int cancoderID) {
        /*
         * 
         * 
         * comment
         * 
         * 
         */
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

        closedLoopReferenceSignal = driveFalcon.getClosedLoopReference();
        closedLoopErrorSignal = driveFalcon.getClosedLoopError();
        closedLoopReferenceSignal.setUpdateFrequency(100);
        closedLoopErrorSignal.setUpdateFrequency(100);

        faultFusedSensorOutOfSync = driveFalcon.getFault_FusedSensorOutOfSync();
        stickyFaultFusedSensorOutOfSync = driveFalcon.getStickyFault_FusedSensorOutOfSync();
        faultRemoteSensorOutOfSync = driveFalcon.getFault_RemoteSensorDataInvalid();
        stickyFaultRemoteSensorOutOfSync = driveFalcon.getStickyFault_RemoteSensorDataInvalid();

        driveMMConfig.MotionMagicCruiseVelocity = 3; // 5 rotations every 1 seconds (defaualt)
        driveMMConfig.MotionMagicAcceleration = 5; // .5 second to reach max speed (defaualt)
        driveMMConfig.MotionMagicJerk = 10; // .33 seconds to reach max accel (defaualt)

        // Feedforward momment!

        // Ya-yoink! (from
        // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
        // zeros the magnet!
        CANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.in(Rotations); // CHANGGE

        driveFalconConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveFalconConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // fuses (trust)
        driveFeedbackConfig.FeedbackRemoteSensorID = driveCancoder.getDeviceID();
        driveFeedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        driveFeedbackConfig.SensorToMechanismRatio = 1.0; // fuck maybe? CHANGE????
        driveFeedbackConfig.RotorToSensorRatio = gearRatio; // CHNAGE

        driveFalconConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveFalconConfig.CurrentLimits.StatorCurrentLimit = 60;

        driveFalconConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        driveFalconConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        driveFalconConfig.Voltage.PeakForwardVoltage = 12;
        driveFalconConfig.Voltage.PeakReverseVoltage = -12;

        drivePID.kP = driveKp;
        drivePID.kI = driveKi;
        drivePID.kD = driveKd;
        drivePID.GravityType = GravityTypeValue.Arm_Cosine;
        drivePID.kA = driveFeedforwardKa; // dont use it (forn now)(trust) (use it ocne sysid works)
        drivePID.kG = driveFeedforwardKg;
        drivePID.kS = driveFeedforwardKs;
        drivePID.kV = driveFeedforwardKv;

        // driveFalcon.setPosition(absoluteEncoderOffset.getRotations());

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = driveFalcon.getConfigurator().apply(driveFalconConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = driveCancoder.getConfigurator().apply(CANcoderConfig);
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
                closedLoopReferenceSignal,
                closedLoopErrorSignal,
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

        inputs.rotorPosition = Rotations.of(rotorPositionSignal.getValueAsDouble());
        inputs.closedLoopReference = Rotations.of(closedLoopReferenceSignal.getValueAsDouble() / gearRatio);
        inputs.closedLoopError = Rotations.of(closedLoopErrorSignal.getValueAsDouble() / gearRatio);

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
    public void setCharacterizationVoltage(Measure<Voltage> volts) {
        driveFalcon.setVoltage(volts.in(Volts)); // needs to counteract gravity
    }

    @Override
    public void setDrivePosition(Measure<Angle> position) {
        // dont let the arm go out of the rage [-30, 270]
        position = Radians.of(MathUtil.clamp(position.in(Radians), -Math.PI / 6.0, 4 * Math.PI / 3.0));

        // lol this is NOT going to work

        driveFalcon.setControl(driveMotionMagic.withPosition(position.in(Rotations)).withSlot(0));
    }
}