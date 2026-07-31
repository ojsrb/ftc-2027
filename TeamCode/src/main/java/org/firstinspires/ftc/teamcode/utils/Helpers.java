package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class Helpers {

    public static class Speeds extends Pose2d {

        /** Center of the field */
        public Speeds() {
            super(0, 0, 0);
        }

        /** y+ = away from red, x+ = right of red perspective */
        public Speeds(double x, double y, double w) {
            super(x, y, w);
        }

        public Speeds toRobot(double deg) {
            return new Speeds(getX() * Math.cos(deg / 180.0 * Math.PI) - getY() * Math.sin(deg / 180.0 * Math.PI), getX() * Math.sin(deg / 180.0 * Math.PI) + getY() * Math.cos(deg / 180.0 * Math.PI), getHeading());
        }
    }

    public static class PoseMeasurement {
        public double timestamp; // in ms
        public Pose2d pose;

        public PoseMeasurement(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    public static class VisionMeasurement extends PoseMeasurement {

        public double confidence;

        public VisionMeasurement(Pose2d pose, double timestamp, double confidence) {
            super(pose, timestamp);
            this.confidence = confidence;
        }
    }

    public static boolean inField(Pose2d pose) {
        return (pose.getX() >= -144.0 && pose.getX() <= 144.0 && pose.getY() >= -144.0 && pose.getY() <= 144.0);
    }
}
