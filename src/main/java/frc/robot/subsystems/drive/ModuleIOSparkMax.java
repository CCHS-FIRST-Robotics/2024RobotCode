package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.Constants;

public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveSparkMax;
    private final CANSparkMax turnSparkMax;
    private final SparkPIDController driveSparkMaxPIDF;
    private final SparkPIDController turnSparkMaxPIDF;
    public int index;

    public double driveKp = 0.00001;
    public double driveKd = 0.0;
    public double driveKi = 0.0;

    public double driveKs = 0.0;
    public double driveKv = 0.136898;
    public double driveKa = 0.020864;

    public double turnKp = 0.00; // 8.00
    public double turnKd = 0.00;
    public double turnKi = 0.00;

    public double turnKs = 0.142578125; // maybe tune a bit more? try between 0.14453125 and 0.140625

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);

    private final RelativeEncoder driveEncoder; // NEO Encoder
    private final RelativeEncoder turnRelativeEncoder; // NEO Encoder
    private final AbsoluteEncoder turnAbsoluteEncoder; // CANandcoder

    // ! figure these out
    private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double turnAfterEncoderReduction = 150.0 / 7.0;
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double couplingRatio = 50d / 14d; // equal to the first stage gear ratio

    private final MedianFilter driveVoltageFilter = new MedianFilter(1000);
    private final MedianFilter turnVoltageFilter = new MedianFilter(1000);

    private Measure<Velocity<Angle>> prevVelocity = RadiansPerSecond.of(0.0);
    private Measure<Angle> prevTurnPosition = Radians.of(0.0);

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

        turnSparkMaxPIDF.setP(turnKp, 0);
        turnSparkMaxPIDF.setD(turnKd, 0);
        turnSparkMaxPIDF.setI(turnKi, 0);
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

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

        driveSparkMax.setInverted(false);
        turnSparkMax.setInverted(true);

        driveSparkMax.setSmartCurrentLimit(50);
        turnSparkMax.setSmartCurrentLimit(30);
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

        turnAbsoluteEncoder.setAverageDepth(2); // NOTE: changed from 8 since last tested run (12/15/23) -- was at 2 a
                                                // couple weeks ago tho

        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        System.out.println(driveSparkMax.burnFlash() == REVLibError.kOk);
        System.out.println(turnSparkMax.burnFlash() == REVLibError.kOk);
    }

    public void setDriveVelocity(Measure<Velocity<Angle>> velocity) {
        driveSparkMaxPIDF.setReference(
                velocity.in(Rotations.per(Minute)) * driveAfterEncoderReduction,
                CANSparkMax.ControlType.kVelocity,
                0,
                driveFeedforward.calculate(
                        prevVelocity.in(RadiansPerSecond),
                        velocity.in(RadiansPerSecond),
                        Constants.PERIOD));
        prevVelocity = velocity;
    }


    double lowKs = 0, highKs = 1, prevRot = -0x123, maxSlope = 1, acc = 1e-10;
    long llt = System.currentTimeMillis(), spacing = 100;
    public void setTurnPosition(Measure<Angle> position) {
        // Adjust from [-PI, PI] (wrapped angle, so initially -pi was 2pi) -> [0, 2PI]
        position = Radians.of(
                MathUtil.inputModulus(position.in(Radians), 0, 2 * Math.PI));

        double p = position.in(Radians);
        double q = prevTurnPosition.in(Radians);
        double π = Math.PI;
        // trust
        int signum = 
            p > q ? 
            p < q + π ? 1 : -1 : 
            p < q ? 
            q < p + π ? 1 : -1 : 
            0;
        System.out.println(sbmstit(p));
        turnSparkMaxPIDF.setReference(
            position.in(Rotations),
            CANSparkMax.ControlType.kPosition,
            0,
            1 * turnKs);

        prevTurnPosition = position;
    }

    public double sbmstit(double cur){
        if(prevRot == -0x123){
            llt = System.currentTimeMillis();
            prevRot = cur;
        }
        if(System.currentTimeMillis() - llt >= spacing){
            llt = System.currentTimeMillis();
            prevRot += prevRot < cur ? 2 * Math.PI : 0;
            if((cur - prevRot) / (System.currentTimeMillis() - llt) < maxSlope){
                lowKs = turnKs;
            }else{
                highKs = turnKs - acc;
            }
            turnKs = lowKs + (highKs - lowKs + 1)/2;
        }
        return turnKs;
    }

    public void setDriveBrakeMode(boolean enable) {
        driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void setTurnBrakeMode(boolean enable) {
        turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        // update drive motor info
        inputs.driveRawPositionRad = // doesnt account for coupling
                Rotations.of(driveEncoder.getPosition() / driveAfterEncoderReduction);
        inputs.drivePositionRad = Rotations.of((driveEncoder.getPosition()
                + turnRelativeEncoder.getPosition() / turnAfterEncoderReduction * couplingRatio)
                / driveAfterEncoderReduction);
        inputs.driveVelocityRadPerSec = Rotations.per(Minute).of(driveEncoder.getVelocity()
                / driveAfterEncoderReduction);
        inputs.driveAppliedVolts = Volts.of(driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage());
        inputs.driveAverageBusVoltage = Volts.of(driveVoltageFilter.calculate(driveSparkMax.getBusVoltage()));
        inputs.driveCurrentAmps = Amps.of(driveSparkMax.getOutputCurrent());
        inputs.driveTempCelcius = Celsius.of(driveSparkMax.getMotorTemperature());

        // update turning motor info
        // System.out.println(turnAbsoluteEncoder.getPosition().getValue());
        inputs.turnAbsolutePositionRad = Radians.of(
                MathUtil.angleModulus(
                        new Rotation2d(
                                turnAbsoluteEncoder.getPosition() // POSITION IN ROTATIONS
                                        * 2 * Math.PI)
                                .getRadians()));
        // inputs.turnAbsolutePositionRad = turnAbsoluteEncoder.getPosition();

        inputs.turnPositionRad = Rotations.of(turnRelativeEncoder.getPosition()
                / turnAfterEncoderReduction);
        inputs.turnVelocityRadPerSec = Rotations.per(Minute).of(turnRelativeEncoder.getVelocity()
                / turnAfterEncoderReduction);
        inputs.turnAppliedVolts = Volts.of(turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage());
        inputs.turnAverageBusVoltage = Volts.of(turnVoltageFilter.calculate(turnSparkMax.getBusVoltage()));
        inputs.turnCurrentAmps = Amps.of(turnSparkMax.getOutputCurrent());
        inputs.turnTempCelcius = Celsius.of(turnSparkMax.getMotorTemperature());
    }
}