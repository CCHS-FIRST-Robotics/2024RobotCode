package frc.robot.subsystems.drive;


import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module{
    private CANSparkMax driveMotor, turnMotor;
    private SparkMaxPIDController pidDriveController, pidTurnController;
    private Canandcoder absoluteTurnEncoder;
    private RelativeEncoder relativeDriveEncoder, relativeTurnEncoder;
    private double driveKP, driveKI, driveKD, driveKIprivate, driveKIz,
             driveKFF, driveKMaxOutput, driveKMinOutput;
    private double turnKP, turnKI, turnKD, turnKIzprivate, turnKIz,
                turnKFF, turnKMaxOutput, turnKMinOutput;
    

    public Module() {
        driveMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        absoluteTurnEncoder = new Canandcoder(2);
        relativeDriveEncoder = driveMotor.getEncoder();
        relativeTurnEncoder = turnMotor.getEncoder();
        driveKP = 0.1; 
        driveKI = 0;
        driveKD = 0; 
        driveKIz = 0; 
        driveKFF = 0;
        driveKMaxOutput = 1; 
        driveKMinOutput = -1;

        turnKP = 0.1; 
        turnKI = 0;
        turnKD = 0;
        turnKIz = 0; 
        turnKFF = 0; 
        turnKMaxOutput = 1; 
        turnKMinOutput = -1;
        
        pidDriveController = driveMotor.getPIDController();
        pidDriveController.setFeedbackDevice(relativeDriveEncoder);
        

        pidDriveController.setP(driveKP);
        pidDriveController.setI(driveKI);
        pidDriveController.setD(driveKD);
        pidDriveController.setIZone(driveKIz); 
        pidDriveController.setFF(driveKFF); 
        pidDriveController.setOutputRange(driveKMinOutput, driveKMaxOutput); 

        pidTurnController.setP(turnKP);
        pidTurnController.setI(turnKI);
        pidTurnController.setD(turnKD);
        pidTurnController.setIZone(turnKIz); 
        pidTurnController.setFF(turnKFF); 
        pidTurnController.setOutputRange(turnKMinOutput, turnKMaxOutput); 


    }

    public void periodic(){
        // Feed forward loop (trust) (wtf does this mean?)
        driveMotor.set(driveKFF);
        turnMotor.set(turnKFF);
    }


    public void driveMotors(SwerveModuleState sms){

        // Swerve
        pidDriveController.setReference(
            metersPerSecondToRotationsPerSecond(sms.speedMetersPerSecond, 0.25), 
            ControlType.kVelocity);
        pidTurnController.setReference(sms.angle.getRotations(), ControlType.kPosition);


    }

    public double metersPerSecondToRotationsPerSecond(double metersPerSecond, double wheelRadiusMeters){
        return 60 * metersPerSecond / (2 * Math.PI * wheelRadiusMeters);
    }
}
/*
Goals:
 
Take state and optimize angle based on function

Gives us new things

Take otuputs, usedPID to set turn motor to that angle and velocity

Take swerve modulo states setpoint, and have motors try to get that setbase

Use .set to setpoint
1 = 12 voltes, 0 is 0 volts, linear scale, 0.5 = 6 volts
"Feed fowrward control
Really simple contral, no feedback

setReference within pidController

Check REV API docs

Just using PID, what would happen

Look at REDUX/Revlib
"Sparkmax Absoluyte Encoder"

Need to put smthn in Drive.java
"Set the speed of the whole chasiy"

ChassisSpeeds Object = speed of whole robot

 Periodic loop of Drive.java, referencwe set point, based on that, we calcualte what states of modulo should be,
 refercen drive motors function.
 */