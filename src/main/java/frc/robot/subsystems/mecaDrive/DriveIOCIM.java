package frc.robot.subsystems.mecaDrive;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;

public class DriveIOCIM implements DriveIO {
  private final TalonSRX frMotor;
  private final TalonSRX flMotor;
  private final TalonSRX brMotor;
  private final TalonSRX blMotor;

  private final AHRS imu;

    private final double kP = 0.1; // idk it was taken from some phoenix example, should double check
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kV = 1023.0/20660.0; // idk it was taken from some phoenix example, should double check

  public DriveIOCIM() {
    frMotor = new TalonSRX(2);
    flMotor = new TalonSRX(10);
    brMotor = new TalonSRX(5);
    blMotor = new TalonSRX(11);

    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.peakCurrentLimit = 40; // the peak current, in amps
    config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
    config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered

    SlotConfiguration pidf = new SlotConfiguration();
    pidf.kP = kP;
    pidf.kI = kI;
    pidf.kD = kD;
    pidf.kF = kV;

    config.slot0 = pidf;

    flMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
    frMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
    brMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
    blMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

    frMotor.setInverted(false);
    flMotor.setInverted(true);
    brMotor.setInverted(false);
    blMotor.setInverted(true);

    imu = new AHRS();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.frPositionRaw = frMotor.getSelectedSensorPosition();
    inputs.frPositionRaw = flMotor.getSelectedSensorPosition();
    inputs.frPositionRaw = brMotor.getSelectedSensorPosition();
    inputs.frPositionRaw = blMotor.getSelectedSensorPosition();

    inputs.frVelocityRaw = frMotor.getSelectedSensorVelocity();
    inputs.flVelocityRaw = flMotor.getSelectedSensorVelocity();
    inputs.brVelocityRaw = brMotor.getSelectedSensorVelocity();
    inputs.blVelocityRaw = blMotor.getSelectedSensorVelocity();
    
    inputs.gyroConnected = imu.isConnected();
    inputs.gyroYawRad = imu.getYaw() * (2 * Math.PI / 180d);
    inputs.gyroYawVelocity = imu.getRate() * (Math.PI / 180d);
  }

  public void setVoltage(double frVolts, double flVolts, double brVolts, double blVolts) {
    frMotor.set(TalonSRXControlMode.PercentOutput, frVolts / 12);
    flMotor.set(TalonSRXControlMode.PercentOutput, flVolts / 12);
    brMotor.set(TalonSRXControlMode.PercentOutput, brVolts / 12);
    blMotor.set(TalonSRXControlMode.PercentOutput, blVolts / 12);
  }

    public void setVelocity(double frVelocity, double flVelocity, double brVelocity, double blVelocity) {
      setVoltage(
        frVelocity / 3.0 * 12, 
        flVelocity / 3.0 * 12, 
        brVelocity / 3.0 * 12, 
        blVelocity / 3.0 * 12
      );
        // frMotor.set(TalonSRXControlMode.Velocity, frVelocity);
        // flMotor.set(TalonSRXControlMode.Velocity, flVelocity);
        // brMotor.set(TalonSRXControlMode.Velocity, brVelocity);
        // blMotor.set(TalonSRXControlMode.Velocity, blVelocity);
    }
}
