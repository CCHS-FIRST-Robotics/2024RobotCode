package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class QuadraticProfile {
    double period;

    public QuadraticProfile() {
        this(.02);
    }

    public QuadraticProfile(double period) {
        this.period = period;
    }

    // public ArrayList<double[]> getSetPoints(Translation2d initialPosition,
    // Translation2d goal, double theta, double speed, double acceleration,
    // ArrayList<Translation2d> guides) {
    // if (guides.size() == 0) return getSetPoints(initialPosition, goal, theta,
    // speed, acceleration);

    // ArrayList<double[]> setpoints = new ArrayList<double[]> ();
    // for (int i=0; i < guides.size(); i++) {
    // Translation2d guide = guides.get(i);
    // if (i == 0) {
    // setpoints.addAll(getSetPoints(initialPosition, guide, theta, speed,
    // acceleration));
    // } else {
    // setpoints.addAll(getSetPoints(guides.get(i-1), guide, theta, speed,
    // acceleration));
    // }
    // }
    // setpoints.addAll(getSetPoints(guides.get(guides.size()-1), goal, theta,
    // speed, acceleration));
    // return setpoints;
    // }

    public Translation2d[] getCombinedSetPoints(Translation2d initialPosition, Translation2d goal, double speed,
            double acceleration) {
        Translation2d[] accelSetpoints, stoppingSetpoints, constantSpeedSetpoints;

        // displacement from the initial (x, y) to goal
        Translation2d displacement = goal.minus(initialPosition);
        double angleDisplacement = displacement.getAngle().getRadians(); // radians

        double timeToAccelerate = speed / acceleration;

        // check if we'll go past the setpoint/can't achieve max velocity and slow down
        // in time
        if (0.5 * acceleration * Math.pow(timeToAccelerate, 2) > 0.5 * displacement.getNorm()) {
            // calculate time to go halfway
            timeToAccelerate = Math.sqrt(displacement.getNorm() / acceleration);

            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement, acceleration);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement,
                    acceleration * timeToAccelerate, acceleration);

            // never reach max velocity so there should be no setpoints at a constant
            // velocity
            Translation2d[] temp = { new Translation2d(0, 0) };
            constantSpeedSetpoints = temp;
        } else {
            accelSetpoints = getAcceleratingSetPoints(timeToAccelerate, angleDisplacement, acceleration);
            stoppingSetpoints = getStoppingSetpoints(timeToAccelerate, angleDisplacement,
                    acceleration * timeToAccelerate, acceleration);

            Translation2d lastAccelPoint = accelSetpoints[accelSetpoints.length - 1];
            Translation2d constantSpeedDisplacement = displacement.minus(lastAccelPoint.times(2));
            double timeConstantSpeed = constantSpeedDisplacement.getNorm() / speed;
            constantSpeedSetpoints = getConstantSpeedSetpoints(timeConstantSpeed, angleDisplacement, speed);
        }

        System.out.println("#Accel: " + accelSetpoints.length);
        System.out.println("#Speed: " + constantSpeedSetpoints.length);
        System.out.println("#Stop: " + stoppingSetpoints.length);

        return combineSetPoints(accelSetpoints, constantSpeedSetpoints, stoppingSetpoints, initialPosition);
    }

    // public ArrayList<double[]> getSetPoints(Translation2d initialPosition,
    // Translation2d goal, double theta, double speed, double acceleration) {
    // double directionX, directionY;

    // Translation2d[] combined = getCombinedSetPoints(initialPosition, goal, theta,
    // speed, acceleration);

    // ArrayList<double[]> setpoints = new ArrayList<double[]>(combined.length);
    // double[] angles;
    // for (int i = 0; i < combined.length; i++) {
    // try {
    // double x = combined[i].x;
    // double y = combined[i].y;
    // // System.out.println("(x, y) = " + x + ", " + y);
    // if (i == 0) {
    // angles = Kinematics.positionInverseKinematics(x, y, true);
    // // angles = Kinematics.positionInverseKinematics(x, y, initialAngles);
    // } else {
    // angles = Kinematics.positionInverseKinematics(x, y, true);
    // // angles = Kinematics.positionInverseKinematics(x, y, setpoints.get(i-1));
    // }

    // double wristPosition = Kinematics.wristDesiredPosition(x, y);
    // // if its in between the two thresholds, assume it's flush with upper arm
    // if (wristPosition == -1) {
    // wristPosition = 0;
    // }

    // if (i != 0) {
    // double prevX = combined[i-1].x;
    // double prevY = combined[i-1].y;

    // directionX = x - prevX;
    // directionY = y - prevY;

    // if (Kinematics.isMovingPastLimit(Math.toDegrees(angles[0]),
    // Math.toDegrees(angles[1]), wristPosition, directionX, directionY)) {
    // // throw new Exception("(x, y) = (" + x + ", " + y + "), (a, b) = (" +
    // angles[0] + ", " + angles[1] + ") " + "goes past a motor limit");
    // }
    // }

    // if (!Kinematics.isPositionPossible(x, y)) {
    // // throw new ArithmeticException("(x, y) = (" + x + ", " + y + "), (a, b) =
    // (" + angles[0] + ", " + angles[1] + ") " + "is not physically possible");
    // }
    // if (Double.isNaN(angles[0]) || Double.isNaN(angles[1])) {
    // throw new ArithmeticException("SOMETHING MESSED UP - angle is NaN");
    // }
    // } catch(ArithmeticException e)
    // {
    // // System.out.println("X: " + combined[i].x);
    // // System.out.println("Y: " + combined[i].y);
    // System.out.println(e.getMessage());
    // if (i != 0) {
    // setpoints.add(setpoints.get(i-1));
    // }
    // continue;
    // } catch(Exception e) {
    // // If the motor is past a limit, stop the sequence
    // // System.out.println("X: " + combined[i].x);
    // // System.out.println("Y: " + combined[i].y);
    // System.out.println(e.getMessage());

    // break;
    // }

    // setpoints.add(angles);
    // }
    // System.out.println("GOT HERE -1");
    // return setpoints;
    // }

    public Translation2d[] getAcceleratingSetPoints(double timeToEnd, double angle, double acceleration) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Translation2d[] angles = new Translation2d[numSteps];

        for (int i = 0; i < numSteps; i++) {
            angles[i] = new Translation2d(
                    0.5 * Math.cos(angle) * acceleration * Math.pow(i * period, 2),
                    0.5 * Math.sin(angle) * acceleration * Math.pow(i * period, 2));
        }
        return angles;
    }

    public Translation2d[] getStoppingSetpoints(double timeToEnd, double angle, double velocity, double acceleration) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Translation2d[] angles = new Translation2d[numSteps];

        for (int i = 0; i < numSteps; i++) {
            double dt = i * period;
            angles[i] = new Translation2d(
                    (velocity - 0.5 * acceleration * dt) * Math.cos(angle) * dt,
                    (velocity - 0.5 * acceleration * dt) * Math.sin(angle) * dt);
        }
        return angles;
    }

    public Translation2d[] getConstantSpeedSetpoints(double timeToEnd, double angle, double velocity) {
        int numSteps = (int) Math.ceil(timeToEnd / period);
        Translation2d[] angles = new Translation2d[numSteps];

        for (int i = 0; i < numSteps; i++) {
            angles[i] = new Translation2d(
                    (velocity * i * period) * Math.cos(angle),
                    (velocity * i * period) * Math.sin(angle));
        }
        return angles;
    }

    public Translation2d[] combineSetPoints(Translation2d[] accel, Translation2d[] constant, Translation2d[] stopping,
            Translation2d initialPosition) {
        Translation2d[] combined = new Translation2d[accel.length + constant.length + stopping.length];

        for (int i = 0; i < accel.length; i++) {
            combined[i] = accel[i].plus(initialPosition);
        }
        for (int i = 0; i < constant.length; i++) {
            combined[i + accel.length] = constant[i].plus(combined[accel.length - 1]);
        }
        for (int i = 0; i < stopping.length; i++) {
            combined[i + accel.length + constant.length] = stopping[i]
                    .plus(combined[accel.length + constant.length - 1]);
        }

        return combined;
    }
}