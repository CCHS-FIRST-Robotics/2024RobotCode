package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    
    CameraIO io;
    CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public Vision(CameraIO io) {
        this.io = io;
    }
    
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);
    }

    public Transform2d getTransformToClosestTag() {
        return new Transform2d(new Translation2d(inputs.primaryTagX, inputs.primaryTagY), new Rotation2d(inputs.primaryTagHeading));
    }
}
