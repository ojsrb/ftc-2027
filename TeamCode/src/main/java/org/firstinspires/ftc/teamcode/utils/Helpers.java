package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class Helpers {

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
