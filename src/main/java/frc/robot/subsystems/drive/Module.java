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

    private double WHEEL_RADIUS = .0508;
    private double GEAR_RATIO = 6.75;

    public Module(int idNum) {

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
        turnPIDController.setFeedbackDevice(turnAbsoluteEncoder);

        // drive PID coefficients
        drivekP = 0;
        drivekI = 0;
        drivekD = 0; // keep 0
        drivekIz = 0;
        drivekFF = 0;
        drivekMaxOutput = 1;
        drivekMinOutput = -1;

        // turn PID coefficients
        turnkP = 0;
        turnkI = 0; // keep 0
        turnkD = 0; // keep 0
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

    public void setReferences(SwerveModuleState state) {
        turnPIDController.setReference(state.angle.getRotations(), ControlType.kVelocity);
        drivePIDController.setReference(metersPerSecondToRPM(state.speedMetersPerSecond), ControlType.kVelocity);
    }

    public double metersPerSecondToRPM(double metersPerSecond) {
        return GEAR_RATIO * metersPerSecond * 60 / (2 * Math.PI * WHEEL_RADIUS);
    }

}
