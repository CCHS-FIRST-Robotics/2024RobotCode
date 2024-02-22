package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.subsystems.noteIO.arm.Arm;

public class ArmControlWithJoysticks extends Command {
    Arm arm;
    Supplier<Double> linearXSupplier;
    Supplier<Double> linearYSupplier;
    Supplier<Double> angularSupplier;

    public ArmControlWithJoysticks(Arm a, Supplier<Double> x, Supplier<Double> y, Supplier<Double> theta) {
        addRequirements(a);
        arm = a;
        linearXSupplier = x;
        linearYSupplier = y;
        angularSupplier = theta;
    }

    @Override
    public void execute() {
        double controllerX = linearXSupplier.get();
        double controllerY = linearYSupplier.get();
        double controllerAngle = angularSupplier.get();
        
        // lol filler code
    }
}