package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.geometry.Pose2d;

import java.util.List;

public class Helpers {

    public static class PoseMeasurement {
        public final double timestamp; // in ms
        public Pose2d pose;

        public PoseMeasurement(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    public static class VisionMeasurement extends PoseMeasurement {

        public final double confidence;

        public VisionMeasurement(Pose2d pose, double timestamp, double confidence) {
            super(pose, timestamp);
            this.confidence = confidence;
        }
    }

    public static boolean inField(Pose2d pose) {
        return (pose.getX() >= -144.0 && pose.getX() <= 144.0 && pose.getY() >= -144.0 && pose.getY() <= 144.0);
    }

    public static VisionMeasurement averageVisionMeasurements(List<VisionMeasurement> measurements) {
        double xSum = 0.0;
        double ySum = 0.0;
        double wSum = 0.0;
        double weightSum = 0.0;
        for (VisionMeasurement visionMeasurement: measurements) {
            xSum += visionMeasurement.pose.getX() * visionMeasurement.confidence;
            ySum += visionMeasurement.pose.getY() * visionMeasurement.confidence;
            wSum += visionMeasurement.pose.getHeading() * visionMeasurement.confidence;
            weightSum += visionMeasurement.confidence;
        }
        return new VisionMeasurement(new Pose2d(xSum / weightSum, ySum / weightSum, wSum / weightSum), measurements.get(0).timestamp, weightSum / measurements.size());

    }

}
