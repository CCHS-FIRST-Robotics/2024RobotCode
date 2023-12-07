package frc.robot.subsystems.drive;


import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Module{
    public CANSparkMax driveMotor, turnMotor;
    public SparkMaxPIDController pidDriveController, pidTurnController;

    public Canandcoder absoluteTurnEncoder;

    public RelativeEncoder relativeDriveEncoder, relativeTurnEncoder;

    public double driveKP, driveKI, driveKD, driveKIz,
                driveKFF, driveKMaxOutput, driveKMinOutput;

    public double turnKP, turnKI, turnKD, turnKIz, 
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
        
        // sure

    }

}