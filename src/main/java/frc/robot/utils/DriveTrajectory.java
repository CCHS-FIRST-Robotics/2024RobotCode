package frc.robot.utils;

import edu.wpi.first.math.geometry.*;
import java.io.*;
import java.util.ArrayList;

import com.opencsv.CSVWriter;

public class DriveTrajectory {

    public ArrayList<Pose2d> positionTrajectory;
    public ArrayList<Twist2d> velocityTrajectory;

    public DriveTrajectory() {
        this.positionTrajectory = new ArrayList<Pose2d>();
        this.velocityTrajectory = new ArrayList<Twist2d>();
    }

    public DriveTrajectory(ArrayList<Pose2d> positionTrajectory, ArrayList<Twist2d> velocityTrajectory) {
        this.positionTrajectory = positionTrajectory;
        this.velocityTrajectory = velocityTrajectory;
    }

    public void add(DriveTrajectory other) {
        ArrayList<Pose2d> combinedPositionTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> combinedVelocityTrajectory = new ArrayList<Twist2d>();

        combinedPositionTrajectory.addAll(this.positionTrajectory);
        combinedPositionTrajectory.addAll(other.positionTrajectory);

        combinedVelocityTrajectory.addAll(this.velocityTrajectory);
        combinedVelocityTrajectory.addAll(other.velocityTrajectory);

        positionTrajectory = combinedPositionTrajectory;
        velocityTrajectory = combinedVelocityTrajectory;
    }

    public static DriveTrajectory combine(DriveTrajectory first, DriveTrajectory second) {
        ArrayList<Pose2d> combinedPositionTrajectory = new ArrayList<Pose2d>();
        ArrayList<Twist2d> combinedVelocityTrajectory = new ArrayList<Twist2d>();

        combinedPositionTrajectory.addAll(first.positionTrajectory);
        combinedPositionTrajectory.addAll(second.positionTrajectory);

        combinedVelocityTrajectory.addAll(first.velocityTrajectory);
        combinedVelocityTrajectory.addAll(second.velocityTrajectory);

        return new DriveTrajectory(combinedPositionTrajectory, combinedVelocityTrajectory);
    }

    public DriveTrajectory combine(DriveTrajectory other) {
        return combine(this, other);
    }

    public void translateBy(Translation2d translation) {
        for (int i = 0; i < this.positionTrajectory.size(); i++) {
            Pose2d pose = this.positionTrajectory.get(i);
            this.positionTrajectory.set(i, new Pose2d(pose.getTranslation().plus(translation), pose.getRotation()));
        }
    }

    public void toCSV() {
        toCSV("trajectory");
    }

    public void toCSV(String filename) {
        try {
            CSVWriter writer = new CSVWriter(new FileWriter("data/" + filename + ".csv"));
            String[] header = { "x", "y", "heading", "dx", "dy", "dtheta" };
            writer.writeNext(header);
            for (int i = 0; i < this.positionTrajectory.size(); i++) {
                Pose2d pose = this.positionTrajectory.get(i);
                Twist2d twist = this.velocityTrajectory.get(i);
                String[] data = {
                        Double.toString(pose.getX()),
                        Double.toString(pose.getY()),
                        Double.toString(pose.getRotation().getRadians()),
                        Double.toString(twist.dx), Double.toString(twist.dy),
                        Double.toString(twist.dtheta)
                };
                writer.writeNext(data);
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
