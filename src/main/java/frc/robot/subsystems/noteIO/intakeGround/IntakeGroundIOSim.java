package frc.robot.subsystems.noteIO.intakeGround;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeGroundIOSim implements IntakeGroundIO {
    // CANSparkMax motor1 = new CANSparkMax();
    // CANSparkMax motor2 = new CANSparkMax();
    // RelativeEncoder encoder1 = motor1.getEncoder();
    // RelativeEncoder encoder2 = motor1.getEncoder();

    public IntakeGroundIOSim() {

    }

    public void updateInputs(IntakeGroundIOInputs inputs) {
        // inputs.motor1Voltage = motor1.getBusVoltage();
        // inputs.motor1Current = motor1.getOutputCurrent();
        // inputs.motor1Velocity = encoder1.getVelocity();
        // inputs.motor1Temperature = motor1.getMotorTemperature();

        // inputs.motor2Voltage = motor2.getBusVoltage();
        // inputs.motor2Current = motor2.getOutputCurrent();
        // inputs.motor2Velocity = encoder2.getVelocity();
        // inputs.motor2Temperature = motor2.getMotorTemperature();
    }

    public void setVoltage(double volts) {
        // motor1.setVoltage(volts);
        // motor2.setVoltage(volts);
    }
}