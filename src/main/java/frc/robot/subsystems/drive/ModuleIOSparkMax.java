package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;

    private double driveKp = 0.00001;
    private double driveKi = 0;
    private double driveKd = 0;
    private double driveKs = 0;
    private double driveKv = 0.136898;
    private double driveKa = 0.020864;

    private double turnKp = 8;
    private double turnKd = 0;

    private final SparkPIDController driveSparkMaxPIDF;
    private final SimpleMotorFeedforward driveFeedforward;
    private final SparkPIDController turnSparkMaxPIDF;

    private final RelativeEncoder driveEncoder; // NEO
    private final RelativeEncoder turnRelativeEncoder; // NEO
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandcoder

    // trust in colin's math! 
    private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double turnAfterEncoderReduction = 150.0 / 7.0;
    private final double couplingRatio = 50.0 / 14.0;

    private Measure<Velocity<Angle>> prevDriveVelocity = RadiansPerSecond.of(0.0);

    int index;

    public ModuleIOSparkMax(int index) {
        this.index = index;

        driveSparkMax = new CANSparkMax(2 + 2 * index, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(1 + 2 * index, MotorType.kBrushless);

        driveSparkMaxPIDF = driveSparkMax.getPIDController();
        turnSparkMaxPIDF = turnSparkMax.getPIDController();

        driveSparkMaxPIDF.setP(driveKp, 0);
        driveSparkMaxPIDF.setI(driveKi, 0);
        driveSparkMaxPIDF.setD(driveKd, 0);
        driveSparkMaxPIDF.setFF(0, 0);
        driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

        turnSparkMaxPIDF.setP(turnKp, 0);
        turnSparkMaxPIDF.setI(0, 0);
        turnSparkMaxPIDF.setD(turnKd, 0);
        turnSparkMaxPIDF.setFF(0, 0);

        turnSparkMaxPIDF.setPositionPIDWrappingEnabled(true);
        turnSparkMaxPIDF.setPositionPIDWrappingMinInput(0);
        turnSparkMaxPIDF.setPositionPIDWrappingMaxInput(1);

        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        turnAbsoluteEncoder.setInverted(true);
        turnSparkMaxPIDF.setFeedbackDevice(turnAbsoluteEncoder);

        driveSparkMax.setCANTimeout(500);
        turnSparkMax.setCANTimeout(500);

        driveEncoder = driveSparkMax.getEncoder();
        turnRelativeEncoder = turnSparkMax.getEncoder();

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        turnSparkMax.setInverted(true);

        driveSparkMax.setSmartCurrentLimit(30);
        turnSparkMax.setSmartCurrentLimit(20);
        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        driveSparkMax.setIdleMode(IdleMode.kBrake);
        turnSparkMax.setIdleMode(IdleMode.kBrake);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        turnRelativeEncoder.setPosition(0.0);
        turnRelativeEncoder.setMeasurementPeriod(10);
        turnRelativeEncoder.setAverageDepth(2);
        turnRelativeEncoder.setPositionConversionFactor(1);

        turnAbsoluteEncoder.setAverageDepth(2);

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }
    
    public void setDriveVoltage(Measure<Voltage> volts) {
        driveSparkMax.setVoltage(volts.in(Volts));
    }
    
    public void setTurnVoltage(Measure<Voltage> volts) {
        turnSparkMax.setVoltage(volts.in(Volts));
    }
    
    public void setDriveVelocity(Measure<Velocity<Angle>> velocity) {
        driveSparkMaxPIDF.setReference(
            velocity.in(Rotations.per(Minute)) * driveAfterEncoderReduction,
            CANSparkMax.ControlType.kVelocity,
            0,
            driveFeedforward.calculate(
                prevDriveVelocity.in(RadiansPerSecond),
                velocity.in(RadiansPerSecond),
                Constants.PERIOD
            )
        );

        prevDriveVelocity = velocity;
    }

    public void setTurnPosition(Measure<Angle> position) {
        // Adjust from [-PI, PI] -> [0, 2PI]
        position = Radians.of(MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));
            
        turnSparkMaxPIDF.setReference(
            position.in(Rotations),
            CANSparkMax.ControlType.kPosition,
            0
        );
    }

    // ! idk if these are necessary
    public void setDriveBrakeMode(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }
    
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveRawPositionRad = // doesnt account for coupling
            Rotations.of(driveEncoder.getPosition() / driveAfterEncoderReduction
        );
        inputs.drivePositionRad = 
            Rotations.of((driveEncoder.getPosition()
            + turnRelativeEncoder.getPosition() / turnAfterEncoderReduction * couplingRatio)
            / driveAfterEncoderReduction
        );
        inputs.driveVelocityRadPerSec = 
            Rotations.per(Minute).of(driveEncoder.getVelocity()
            / driveAfterEncoderReduction
        );
        inputs.driveAppliedVolts = Volts.of(driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage());
        inputs.driveCurrentAmps = Amps.of(driveSparkMax.getOutputCurrent());
        inputs.driveTempCelcius = Celsius.of(driveSparkMax.getMotorTemperature());

        inputs.turnAbsolutePositionRad = Radians.of(
                MathUtil.angleModulus(
                        new Rotation2d(
                                turnAbsoluteEncoder.getPosition() // POSITION IN ROTATIONS
                                        * 2 * Math.PI)
                                .getRadians()));

        inputs.turnPositionRad = Rotations.of(turnRelativeEncoder.getPosition()
                / turnAfterEncoderReduction);
        inputs.turnVelocityRadPerSec = Rotations.per(Minute).of(turnRelativeEncoder.getVelocity()
                / turnAfterEncoderReduction);
        inputs.turnAppliedVolts = Volts.of(turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage());
        inputs.turnCurrentAmps = Amps.of(turnSparkMax.getOutputCurrent());
        inputs.turnTempCelcius = Celsius.of(turnSparkMax.getMotorTemperature());
    }
}