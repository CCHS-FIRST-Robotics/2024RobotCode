package frc.robot.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.*;

public class SwerveKinematicUtils {
    
    /**
     * Calculates the module acclerations based on 2nd order kinematics
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     *                               
     *                               | ax |
     * | amx |   |  1  0 -rx -ry | * | ay |
     * | amy | = |  0  1 -ry  rx |   | w^2 |
     *                               | alpha |
     * 
     * @param modulePosition
     * @param chassisAcceleration
     * @param chassisAngularVelocity
     * @return
     */
    public static Translation2d getModuleAccelerations(Translation2d modulePosition, Twist2d chassisAcceleration, double chassisAngularVelocity) {
        Matrix<N2, N4> A = MatBuilder.fill(Nat.N2(), Nat.N4(),
            1d, 0d, -modulePosition.getX(), -modulePosition.getY(),
            0d, 1d, -modulePosition.getY(), modulePosition.getX()
        );

        Matrix<N4, N1> B = VecBuilder.fill(chassisAcceleration.dx, chassisAcceleration.dy, chassisAngularVelocity * chassisAngularVelocity, chassisAcceleration.dtheta);

        Matrix<N2, N1> X = A.times(B);
        return new Translation2d(X.get(0, 0), X.get(1, 0));
    }
}
