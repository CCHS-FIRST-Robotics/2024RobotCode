package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private CANSparkMax driveMotor, turnMotor;
    private SparkAbsoluteEncoder turnAbsoluteEncoder;
    private RelativeEncoder turnRelativeEncoder, driveRelativeEncoder;
    private SparkPIDController drivePIDController, turnPIDController;
    private double drivekP, drivekI, drivekD, drivekIz, drivekFF, drivekMinOutput, drivekMaxOutput;
    private double turnkP, turnkI, turnkD, turnkIz, turnkFF, turnkMinOutput, turnkMaxOutput;


    public Module(int idNum) {
        
        /* 
        use controller.getEncoder(), returns relative encoder
        .getAbsoluteEncoder​(SparkAbsoluteEncoder.kDutyCycle)

        sparkmax:
        .setVoltage​(double outputVolts) sets voltage output
        .get() gets current set speed

        SwervewModuleState class
        SwerveModuleState​(double speedMetersPerSecond, Rotation2d angle) = constructor
        
        PID controller sparkmax
        CANSparkBase.getPIDController()
        CANSparkMax subclass of CANSparkBase
        driveMotor.getPIDController() --> SparkPIDController 

        .setP​(double gain)
        .setD(double gain)
        .setI​(double gain)
        .setFF​(double gain)
        .setFeedbackDevice​(MotorFeedbackSensor sensor)	Set the controller's feedback device
        closed loop controller = feedback controller
        Interface MotorFeedbackSensor --> implementing classes: abs, rel encoders

        drive motors:
        feedforward, kP
        feedback, PID

        Izone range --> contains the I error

        wrapping around the angle?
        turnPIDController.setPositionPIDWrappingEnabled(boolean); 
        turnPIDController.setPositionPIDWrappingMinInput(min);
        turnPIDController.setPositionPIDWrappingMaxInput(max);

        pid.setreference(value, ControlType.kPosistion;etc)

        */

        // motors
        driveMotor = new CANSparkMax(2 * idNum + 1, CANSparkLowLevel.MotorType.kBrushless);
        turnMotor = new CANSparkMax(2 * idNum + 2, CANSparkLowLevel.MotorType.kBrushless);

        // encoders
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        turnRelativeEncoder = turnMotor.getEncoder();
        driveRelativeEncoder = driveMotor.getEncoder();

        // PID controllers
        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setFeedbackDevice(driveRelativeEncoder);
        
        turnPIDController = turnMotor.getPIDController();
        turnPIDController.setFeedbackDevice(turnAbsoluteEncoder); //?

        // drive PID coefficients
        drivekP = 0; 
        drivekI = 0;
        drivekD = 0; //keep 0
        drivekIz = 0;
        drivekFF = 0;
        drivekMaxOutput = 1; //?
        drivekMinOutput = -1;

        // turn PID coefficients
        turnkP = 0;
        turnkI = 0; //keep 0
        turnkD = 0; //keep 0
        turnkIz = 0;
        turnkFF = 0;
        drivekMaxOutput = 1;
        drivekMinOutput = -1;

        drivePIDController.setP(drivekP);
        drivePIDController.setI(drivekI);
        drivePIDController.setD(drivekD);
        drivePIDController.setIZone(drivekIz);
        drivePIDController.setFF(drivekFF);
        drivePIDController.setOutputRange(drivekMinOutput, drivekMaxOutput);

        turnPIDController.setP(turnkP);
        turnPIDController.setI(turnkI);
        turnPIDController.setD(turnkD);
        turnPIDController.setIZone(turnkIz);
        turnPIDController.setFF(turnkFF);
        turnPIDController.setOutputRange(turnkMinOutput, turnkMaxOutput);
        turnPIDController.setPositionPIDWrappingEnabled(true); 
        turnPIDController.setPositionPIDWrappingMinInput(0);
        turnPIDController.setPositionPIDWrappingMaxInput(1);
    }

    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void drive(SwerveModuleState state) {
        turnPIDController.setReference(state.angle.getRadians(), ControlType.kVelocity); //radians or rotations?
        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

}
