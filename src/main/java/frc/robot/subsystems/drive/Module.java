package frc.robot.subsystems.drive;


// import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module{
    private CANSparkMax driveMotor, turnMotor;
    private SparkPIDController pidDriveController, pidTurnController;
    private SparkAbsoluteEncoder absoluteTurnEncoder;
    private RelativeEncoder relativeDriveEncoder, relativeTurnEncoder;
    private double driveKP, driveKI, driveKD, driveKIprivate, driveKIz,
             driveKFF, driveKMaxOutput, driveKMinOutput;
    private double turnKP, turnKI, turnKD, turnKIzprivate, turnKIz,
                turnKFF, turnKMaxOutput, turnKMinOutput;

    int i;
    

    public Module(int id) {
        driveMotor = new CANSparkMax(2 * id + 2, CANSparkMaxLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(2 * id + 1, CANSparkMaxLowLevel.MotorType.kBrushless);
        absoluteTurnEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        relativeDriveEncoder = driveMotor.getEncoder();
        relativeTurnEncoder = turnMotor.getEncoder();
        driveKP = 0.0002d;
        driveKI = 0d;
        driveKD = 0d; 
        driveKIz = 0d; 
        driveKFF = 0d;
        driveKMaxOutput = 1d; 
        driveKMinOutput = -1d;

        turnKP = 5d; 
        turnKI = 0d;
        turnKD = 0d;
        turnKIz = 0d; 
        turnKFF = 0d; 
        turnKMaxOutput = 1d; 
        turnKMinOutput = -1d;
        
        pidDriveController = driveMotor.getPIDController();
        pidDriveController.setFeedbackDevice(relativeDriveEncoder);

        pidTurnController = turnMotor.getPIDController();
        pidTurnController.setFeedbackDevice(absoluteTurnEncoder);
    
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


        pidTurnController.setPositionPIDWrappingEnabled(true); 
        pidTurnController.setPositionPIDWrappingMinInput(0);
        pidTurnController.setPositionPIDWrappingMaxInput(1);
    }

    public void periodic(){
        // do not trust
        i++;
    }


    public void driveMotors(SwerveModuleState sms){
        if (i % 50 == 0){
            System.out.println("----------");
            System.out.println(metersPerSecondToRotationsPerMinute(sms.speedMetersPerSecond));
            System.out.println(relativeDriveEncoder.getVelocity());
            System.out.println(driveMotor.getAppliedOutput());
        }
        // Swerve
        pidDriveController.setReference(
            metersPerSecondToRotationsPerMinute(sms.speedMetersPerSecond), 
            ControlType.kVelocity);
    
        pidTurnController.setReference(sms.angle.getRotations(), ControlType.kPosition);
            // pidTurnController.setReference(.5d, ControlType.kVoltage);
        // pidTurnController.setReference(.5, ControlType.kPosition);


    }

    public double metersPerSecondToRotationsPerMinute(double metersPerSecond){
        return Drive.Constants.getDriveGearRatio() * 
        Drive.Constants.getSecondsInAMinute() * metersPerSecond / 
        (Drive.Constants.getRadiiInADiameter() * Math.PI * Drive.Constants.getWheelRadiusMeters());
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