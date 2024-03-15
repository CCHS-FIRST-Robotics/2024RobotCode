package frc.robot.subsystems.noteIO.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.*;

public class ShooterIOFalcon500 implements ShooterIO {
    private TalonFX leftShooter, rightShooter;
    private VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private final Slot0Configs drivePID = talonFXConfig.Slot0;

    private static final double driveKp = 0.1;
    private static final double driveKd = 0d;
    private static final double driveKi = 0.0d;

    private static final double driveFeedforwardKs = 0;
    // Units needed are volts * seconds / rotations, max rpm is 6,380
    private static final double driveFeedforwardKv = .125; // 6380 rotaions per minute is 319/3
                                                           // rotations per second
    private static final double driveFeedforwardKa = 0;

    private StatusSignal<Double> voltageSignalLeft;
    private StatusSignal<Double> currentSignalLeft;
    private StatusSignal<Double> positionSignalLeft;
    private StatusSignal<Double> velocitySignalLeft;
    private StatusSignal<Double> temperatureSignalLeft;

    private StatusSignal<Double> voltageSignalRight;
    private StatusSignal<Double> currentSignalRight;
    private StatusSignal<Double> positionSignalRight;
    private StatusSignal<Double> velocitySignalRight;
    private StatusSignal<Double> temperatureSignalRight;

    private StatusSignal<Double> closedLoopReferenceSignal;

    public ShooterIOFalcon500(int id1, int id2) {
        leftShooter = new TalonFX(id1);
        rightShooter = new TalonFX(id2);

        voltageSignalLeft = leftShooter.getMotorVoltage();
        currentSignalLeft = leftShooter.getStatorCurrent();
        currentSignalLeft.setUpdateFrequency(100);
        positionSignalLeft = leftShooter.getPosition();
        velocitySignalLeft = leftShooter.getVelocity();
        temperatureSignalLeft = leftShooter.getDeviceTemp();
        closedLoopReferenceSignal = leftShooter.getClosedLoopReference();

        voltageSignalRight = rightShooter.getMotorVoltage();
        currentSignalRight = rightShooter.getStatorCurrent();
        currentSignalRight.setUpdateFrequency(100);
        positionSignalRight = rightShooter.getPosition();
        velocitySignalRight = rightShooter.getVelocity();
        temperatureSignalRight = rightShooter.getDeviceTemp();

        talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.SupplyCurrentLimit = 60;
        talonFXConfig.CurrentLimits.SupplyTimeThreshold = 1;

        talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        drivePID.kP = driveKp;
        drivePID.kI = driveKi;
        drivePID.kD = driveKd;
        drivePID.kA = driveFeedforwardKa; // dont use it (for now)(trust) (use it ocne sysid works)
        drivePID.kS = driveFeedforwardKs;
        drivePID.kV = driveFeedforwardKv;

        configFalcon(leftShooter, talonFXConfig);
        talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configFalcon(rightShooter, talonFXConfig);
    }

    private void configFalcon(TalonFX falcon, TalonFXConfiguration falconConfig) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = falcon.getConfigurator().apply(falconConfig);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    @Override
    @AutoLogOutput
    public void setVelocity(Measure<Velocity<Angle>> leftTargetVelocity, Measure<Velocity<Angle>> rightTargetVelocity) {
        leftShooter.setControl(velocityControl.withVelocity(leftTargetVelocity.in(RotationsPerSecond)));
        rightShooter.setControl(velocityControl.withVelocity(rightTargetVelocity.in(RotationsPerSecond)));
    }

    @Override
    public void setVoltage(Measure<Voltage> v) {
        leftShooter.setVoltage(v.in(Volts));
        rightShooter.setVoltage(v.in(Volts));
    }

    @Override
    @AutoLogOutput
    public boolean upToSpeed(Measure<Velocity<Angle>> leftTargetVelocity,
            Measure<Velocity<Angle>> rightTargetVelocity) {
        return velocitySignalLeft.getValue() > leftTargetVelocity.in(RotationsPerSecond) * .95
                && velocitySignalRight.getValue() > rightTargetVelocity.in(RotationsPerSecond) * .95;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                voltageSignalLeft,
                currentSignalLeft,
                positionSignalLeft,
                velocitySignalLeft,
                temperatureSignalLeft,
                closedLoopReferenceSignal,
                voltageSignalRight,
                currentSignalRight,
                positionSignalRight,
                velocitySignalRight,
                temperatureSignalRight);

        inputs.leftShooterVoltage = voltageSignalLeft.getValue();
        inputs.leftShooterCurrent = currentSignalLeft.getValue();
        inputs.leftShooterPosition = positionSignalLeft.getValue();
        inputs.leftShooterVelocity = velocitySignalLeft.getValue();
        inputs.leftShooterTemperature = temperatureSignalLeft.getValue();

        inputs.rightShooterVoltage = voltageSignalRight.getValue();
        inputs.rightShooterCurrent = currentSignalRight.getValue();
        inputs.rightShooterPosition = positionSignalRight.getValue();
        inputs.rightShooterVelocity = velocitySignalRight.getValue();
        inputs.rightShooterTemperature = temperatureSignalRight.getValue();

        inputs.closedLoopReference = closedLoopReferenceSignal.getValue();
    }
}