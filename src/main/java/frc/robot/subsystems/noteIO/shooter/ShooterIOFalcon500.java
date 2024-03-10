package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.*;

public class ShooterIOFalcon500 implements ShooterIO {
    private TalonFX motor1, motor2;
    private VelocityVoltage velocityControl = new VelocityVoltage(0);

    private final TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private final Slot0Configs drivePID = talonFXConfig.Slot0;

    private static final double driveKp = 0.3;
    private static final double driveKd = 0d;
    private static final double driveKi = 0.0d;

    private static final double driveFeedforwardKs = 0;
    // Units needed are volts * seconds / rotations, max rpm is 6,380
    private static final double driveFeedforwardKv = .125; // 6380 rotaions per minute is 319/3
                                                                      // rotations per second
    private static final double driveFeedforwardKa = 0;

    private StatusSignal<Double> voltageSignal1;
    private StatusSignal<Double> currentSignal1;
    private StatusSignal<Double> positionSignal1;
    private StatusSignal<Double> velocitySignal1;
    private StatusSignal<Double> temperatureSignal1;
    private StatusSignal<Double> closedLoopReferenceSignal;

    private StatusSignal<Double> voltageSignal2;
    private StatusSignal<Double> currentSignal2;
    private StatusSignal<Double> positionSignal2;
    private StatusSignal<Double> velocitySignal2;
    private StatusSignal<Double> temperatureSignal2;

    public ShooterIOFalcon500(int id1, int id2) {
        motor1 = new TalonFX(id1);
        motor2 = new TalonFX(id2);

        voltageSignal1 = motor1.getMotorVoltage();
        currentSignal1 = motor1.getStatorCurrent();
        currentSignal1.setUpdateFrequency(100);
        positionSignal1 = motor1.getPosition();
        velocitySignal1 = motor1.getVelocity();
        temperatureSignal1 = motor1.getDeviceTemp();
        closedLoopReferenceSignal = motor1.getClosedLoopReference();

        voltageSignal2 = motor2.getMotorVoltage();
        currentSignal2 = motor2.getStatorCurrent();
        currentSignal2.setUpdateFrequency(100);
        positionSignal2 = motor2.getPosition();
        velocitySignal2 = motor2.getVelocity();
        temperatureSignal2 = motor2.getDeviceTemp();

        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = 60;
        talonFXConfig.CurrentLimits.SupplyTimeThreshold = 1;
        

        drivePID.kP = driveKp;
        drivePID.kI = driveKi;
        drivePID.kD = driveKd;
        drivePID.kA = driveFeedforwardKa; // dont use it (forn now)(trust) (use it ocne sysid works)
        drivePID.kS = driveFeedforwardKs;
        drivePID.kV = driveFeedforwardKv;

        motor1.getConfigurator().apply(talonFXConfig);
        motor2.getConfigurator().apply(talonFXConfig);
    }

    @Override
    @AutoLogOutput
    public void setVelocity(Measure<Velocity<Angle>> v) {
        motor1.setControl(velocityControl.withVelocity(v.in(RotationsPerSecond)));
        motor2.setControl(velocityControl.withVelocity(v.in(RotationsPerSecond)));
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        motor1.setVoltage(v.in(Volts));
        motor2.setVoltage(v.in(Volts));
    }

    @Override
    @AutoLogOutput
    public boolean upToSpeed(Measure<Velocity<Angle>> targetVelocity) {
        return velocitySignal1.refresh().getValue() > targetVelocity.in(RotationsPerSecond);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                voltageSignal1,
                currentSignal1,
                positionSignal1,
                velocitySignal1,
                temperatureSignal1,
                closedLoopReferenceSignal,
                voltageSignal2,
                currentSignal2,
                positionSignal2,
                velocitySignal2,
                temperatureSignal2);

        inputs.motor1Voltage = voltageSignal1.getValue();
        inputs.motor1Current = currentSignal1.getValue();
        inputs.motor1Position = positionSignal1.getValue();
        inputs.motor1Velocity = velocitySignal1.getValue();
        inputs.motor1Temperature = temperatureSignal1.getValue();

        inputs.motor2Voltage = voltageSignal2.getValue();
        inputs.motor2Current = currentSignal2.getValue();
        inputs.motor2Position = positionSignal2.getValue();
        inputs.motor2Velocity = velocitySignal2.getValue();
        inputs.motor2Temperature = temperatureSignal2.getValue();

        inputs.closedLoopReference = closedLoopReferenceSignal.getValue();
    }
}