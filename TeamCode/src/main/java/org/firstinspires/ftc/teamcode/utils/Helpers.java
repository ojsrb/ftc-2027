package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class Helpers {

    public static class Speeds {
        private final double x;
        private final double y;
        private final double w;

        /** Center of the field */
        public Speeds() {
            x = 0;
            y = 0;
            w = 0;
        }

        /** y+ = away from red, x+ = right of red perspective */
        public Speeds(double x, double y, double deg) {
            this.x = x;
            this.y = y;
            this.w = deg;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getDeg() {
            return w;
        }

        public Speeds toRobot(double deg) {
            return new Speeds(getX() * Math.cos(deg / 180.0 * Math.PI) - getY() * Math.sin(deg / 180.0 * Math.PI), getX() * Math.sin(deg / 180.0 * Math.PI) + getY() * Math.cos(deg / 180.0 * Math.PI), getDeg());
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
}
