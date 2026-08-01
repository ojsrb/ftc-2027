package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class Units {
    public static double m2in (double meters) {
        return meters * 39.37;
    }

    public static double in2m (double inches) {
        return inches / 39.37;
    }

    public static double rad2deg(double rad) {
        return rad / Math.PI * 180.0;
    }

    public static double deg2rad(double deg) {
        return deg / 180.0 * Math.PI;
    }

    public static double sec2ms(double seconds) {
        return seconds * 1000.0;
    }

    public static double ms2sec(double ms) {
        return ms / 1000.0;
    }

    public static Pose2d m2in(Pose2d pose) {
        return new Pose2d(m2in(pose.getX()), m2in(pose.getY()), rad2deg(pose.getRotation().getRadians()));
    }

}
