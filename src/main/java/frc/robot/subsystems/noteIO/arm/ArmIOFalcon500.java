package frc.robot.subsystems.noteIO.arm;

import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;

public class ArmIOFalcon500 implements ArmIO {
    /* MOTOR CONTROLLERS + PID */
    private final TalonFX leadFalcon;
    private final TalonFX followerFalcon;
    private final TalonFXConfiguration driveFalconConfig = new TalonFXConfiguration();
    CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

    private CANcoder driveCancoder;

    // TODO - USE FOC (Field Oriented Control) - MotionMagicTorqueCurrentFOC
    private final MotionMagicVoltage driveMotionMagicVoltage = new MotionMagicVoltage(0);
    private final MotionMagicTorqueCurrentFOC driveMotionMagicCurrent = new MotionMagicTorqueCurrentFOC(0);
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

    private static final double gearRatio = 100 * 26d / 14d; // 100 * 54 / 15d

    // TODO: update constants in periodic once tunable is set up
    private static final double driveKpV = 0; // 180
    private static final double driveKdV = 0d; // 3
    private static final double driveKiV = 0.0d;

    // Uhh Feedforward momment!
    private static final double driveFeedforwardKgV = 0; // .435V
    private static final double driveFeedforwardKsV = 0;
    // Units needed are volts * seconds / rotations, max rpm is 6,380
    private static final double driveFeedforwardKvV = 12 * (3 / 319d) / gearRatio; // 6380 rotaions per minute is 319/3
                                                                                  // rotations per second
    private static final double driveFeedforwardKaV = 0;

    private static final double driveKpTC = 650; // 620
    private static final double driveKdTC = 1000; // 120
    private static final double driveKiTC = 1d;

    // Uhh Feedforward momment!
    private static final double driveFeedforwardKgTC = 8; // 9.2A
    private static final double driveFeedforwardKsTC = 0;
    private static final double driveFeedforwardKvTC = 0; // only used for viscous friction losses in TC
    private static final double driveFeedforwardKaTC = 0;

    boolean torqueCurrent = true;

    // private final boolean motorInverted = false;
    private final Measure<Angle> absoluteEncoderOffset = Radians.of(4.25); // -3.71

    int index;

    public ArmIOFalcon500(int motor1ID, int motor2ID, int cancoderID) {
        /*
         * 
         * 
         * comment
         * 
         * 
         */
        leadFalcon = new TalonFX(motor1ID);
        followerFalcon = new TalonFX(motor2ID);
        /// I encode???? Turst Different ids needed NO IDEA WHAT IDS
        driveCancoder = new CANcoder(cancoderID);

        drivePositionSignal = leadFalcon.getPosition();
        driveVelocitySignal = leadFalcon.getVelocity();
        driveAppliedVoltageSignal = leadFalcon.getMotorVoltage();
        driveCurrentSignal = leadFalcon.getSupplyCurrent();
        driveTempSignal = leadFalcon.getDeviceTemp();

        absolutePositionSignal = driveCancoder.getPosition(); // check what this does!!!!!!! absolute variations! other
                                                              // methord!!!!
        absoluteVelocitySignal = driveCancoder.getVelocity();
        rotorPositionSignal = leadFalcon.getRotorPosition();

        closedLoopReferenceSignal = leadFalcon.getClosedLoopReference();
        closedLoopErrorSignal = leadFalcon.getClosedLoopError();
        closedLoopReferenceSignal.setUpdateFrequency(100);
        closedLoopErrorSignal.setUpdateFrequency(100);

        faultFusedSensorOutOfSync = leadFalcon.getFault_FusedSensorOutOfSync();
        stickyFaultFusedSensorOutOfSync = leadFalcon.getStickyFault_FusedSensorOutOfSync();
        faultRemoteSensorOutOfSync = leadFalcon.getFault_RemoteSensorDataInvalid();
        stickyFaultRemoteSensorOutOfSync = leadFalcon.getStickyFault_RemoteSensorDataInvalid();

        driveMMConfig.MotionMagicCruiseVelocity = 98d / gearRatio; // max rps of the motor (almost)
        driveMMConfig.MotionMagicAcceleration = 1; // .5 second to reach max speed (defaualt)
        driveMMConfig.MotionMagicJerk = 3; // .33 seconds to reach max accel (defaualt)

        // Feedforward momment!

        // Ya-yoink! (from
        // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
        // zeros the magnet!
        CANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.in(Rotations); // CHANGGE

        driveFalconConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveFalconConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        // fuses (trust)
        driveFeedbackConfig.FeedbackRemoteSensorID = driveCancoder.getDeviceID();
        driveFeedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        driveFeedbackConfig.SensorToMechanismRatio = 1.0; // fuck maybe? CHANGE????
        driveFeedbackConfig.RotorToSensorRatio = gearRatio; // CHNAGE

        // driveFalconConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // driveFalconConfig.CurrentLimits.StatorCurrentLimit = 60;
        driveFalconConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveFalconConfig.CurrentLimits.SupplyCurrentLimit = 60;
        driveFalconConfig.CurrentLimits.SupplyTimeThreshold = .5;

        driveFalconConfig.TorqueCurrent.PeakForwardTorqueCurrent = 90;
        driveFalconConfig.TorqueCurrent.PeakReverseTorqueCurrent = -90;

        driveFalconConfig.Voltage.PeakForwardVoltage = 12;
        driveFalconConfig.Voltage.PeakReverseVoltage = -12;


        drivePID.kP = torqueCurrent ? driveKpTC : driveKpV;
        drivePID.kI = torqueCurrent ? driveKiTC : driveKiV;
        drivePID.kD = torqueCurrent ? driveKdTC : driveKdV;

        drivePID.GravityType = GravityTypeValue.Arm_Cosine;
        drivePID.kA = torqueCurrent ? driveFeedforwardKaTC : driveFeedforwardKaV; // dont use it (forn now)(trust) (use it ocne sysid works)
        drivePID.kG = torqueCurrent ? driveFeedforwardKgTC : driveFeedforwardKgV;
        drivePID.kV = torqueCurrent ? driveFeedforwardKvTC : driveFeedforwardKvV;
        drivePID.kS = torqueCurrent ? driveFeedforwardKsTC : driveFeedforwardKsV;

        // driveFalcon.setPosition(absoluteEncoderOffset.getRotations());

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = leadFalcon.getConfigurator().apply(driveFalconConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = followerFalcon.getConfigurator().apply(driveFalconConfig);
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

        followerFalcon.setControl(
            new Follower(leadFalcon.getDeviceID(), true)
        );
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
        leadFalcon.setVoltage(volts.in(Volts));
    }

    @Override
    public void setDriveCurrent(Measure<Current> current) {
        leadFalcon.setControl(
            new TorqueCurrentFOC(current.in(Amps))
        );
    }

    @Override
    public void setCharacterizationVoltage(Measure<Voltage> volts) {
        leadFalcon.setVoltage(volts.in(Volts)); // needs to counteract gravity
    }

    @Override
    public void setDrivePosition(Measure<Angle> position) {
        // dont let the arm go out of the rage [-30, 270]
        position = Radians.of(MathUtil.clamp(position.in(Radians), -Math.PI / 6.0, 4 * Math.PI / 3.0));

        // lol this is NOT going to work
        if (!torqueCurrent) {
            leadFalcon.setControl(driveMotionMagicVoltage.withPosition(position.in(Rotations)).withSlot(0));
        } else {
            leadFalcon.setControl(driveMotionMagicCurrent.withPosition(position.in(Rotations)).withSlot(0));
        }
            
    }

    @Override
    public void addToOrchestra(Orchestra orchestra, int trackNum) {
        orchestra.addInstrument(leadFalcon, trackNum);
        orchestra.addInstrument(followerFalcon, trackNum + 1);
    }
}