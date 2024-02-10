package frc.robot;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// Create motor objects in Shooter.java, write simple methods for setting the voltage of motors and getting data from encoders.
// Write a method to control the velocity of the motors (feedforward + feedback)
// Write controller bindings to toggle the shooter on/off at a fixed RPM
import com.ctre.phoenix6.hardware.TalonFX;
public class Shooter {
    TalonFX motor1;
    public double radius;
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
        double motorspeed = motor.getValue() * 2 * Math.PI * radius / 10.71;
        return motorspeed;
    }
    public void setVelocity(){
        
    }
}
