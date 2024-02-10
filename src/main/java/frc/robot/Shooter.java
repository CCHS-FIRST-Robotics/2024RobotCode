package frc.robot;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
// Create motor objects in Shooter.java, write simple methods for setting the voltage of motors and getting data from encoders.
// Write a method to control the velocity of the motors (feedforward + feedback)
// Write controller bindings to toggle the shooter on/off at a fixed RPM
import com.ctre.phoenix6.hardware.TalonFX;
public class Shooter {
    TalonFX motor1;
    double kS = 0.0;
    double kV = 0.0;
    double kA = 0.0;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    PIDController pid = new PIDController(kP, kI, kD);

    private double radius;
    public Shooter(){

    }
    public Shooter(TalonFX motor1){
        this.motor1 = motor1;
    }
    public void setVolage(double voltage){
        motor1.setVoltage(voltage);
    }
    public double getVelocity(){
        StatusSignal<Double> motor = motor1.getVelocity();
        double motorspeed = motor.getValue() * 2 * Math.PI * radius;
        return motorspeed;
    }
    public void setVelocity(double targetVelocity){
        double volts = feedforward.calculate(targetVelocity);
        volts += pid.calculate(getVelocity(), targetVelocity);
        motor1.setVoltage(volts);
    }
}
