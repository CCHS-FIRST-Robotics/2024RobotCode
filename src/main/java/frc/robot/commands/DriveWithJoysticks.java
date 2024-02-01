package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithJoysticks extends Command {

  private Drive swerveDrive;
  private ChassisSpeeds setSpeeds;
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;

  private int USB_PORT = 0;

  public DriveWithJoysticks(Drive swerveDrive, Supplier<Double> xSupplier, Supplier<Double> ySupplier) {

    this.swerveDrive = swerveDrive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
      suppliers
      */
    double speed = xSupplier.get();
    Rotation2d angle = new Rotation2d(-Math.PI + (ySupplier.get() * Math.PI * 2));
    Translation2d linearVelocity = new Translation2d(speed * swerveDrive.getMaxLinearSpeed(), angle);
    // this is probably wrong
    swerveDrive.setState(new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), speed * swerveDrive.getMaxAngularSpeed()), angle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
