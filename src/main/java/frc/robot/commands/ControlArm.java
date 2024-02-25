package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.noteIO.arm.Arm;
import java.util.function.Supplier;

public class ControlArm extends Command {
    Arm arm;
    Supplier<Double> angleSupplier;

    public ControlArm(Arm arm, Supplier<Double> angleSupplier) {
        this.arm = arm;
        this.angleSupplier = angleSupplier;
    }

    // @Override
    // public void initialize() {

    // }

    @Override
    public void execute() {
        // angleSupplier is from -1 to 1 (je pense)
        // we want from pi/6 to pi/3 (trust!)
        // -1 maps to pi/6, 1 maps to pi/3
        // assume linear, 0 maps to pi/4
        // trust!
        double start = angleSupplier.get();
        start *= Math.PI / 12D;
        start += Math.PI / 4D;
        arm.setArmAngle(Radians.of(start));
    }

}
