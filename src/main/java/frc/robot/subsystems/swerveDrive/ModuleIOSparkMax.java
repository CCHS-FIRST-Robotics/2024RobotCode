package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;



public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder; // NEO Encoder
  private final RelativeEncoder turnRelativeEncoder; // NEO Encoder
  private final CANcoder turnAbsoluteEncoder; // CANcoder

  private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    System.out.println("[Init] Creating ModuleIOSparkMax " + Integer.toString(index));

    driveSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    // turnAbsoluteEncoder = new AnalogInput(0);
    turnAbsoluteEncoder = new CANcoder(11, "rio");
    absoluteEncoderOffset = new Rotation2d(-3.03887450);

    driveSparkMax.setCANTimeout(500);
    turnSparkMax.setCANTimeout(500);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */

    turnAbsoluteEncoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    turnAbsoluteEncoder.getPosition().setUpdateFrequency(100);
    turnAbsoluteEncoder.getVelocity().setUpdateFrequency(100);

    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

    turnSparkMax.setInverted(isTurnMotorInverted);

    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    // update drive motor info
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

    // update turning motor info
    // System.out.println(turnAbsoluteEncoder.getPosition().getValue());
    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    turnAbsoluteEncoder.getPosition() // POSITION IN ROTATIONS
                    .getValue() * 2 * Math.PI)
                .minus(absoluteEncoderOffset)
                .getRadians());
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnRelativeEncoder.getPosition())
            / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / turnAfterEncoderReduction;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
  }

  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
