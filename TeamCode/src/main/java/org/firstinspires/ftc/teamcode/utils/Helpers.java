package org.firstinspires.ftc.teamcode.utils;

public class Helpers {

    public static class Translation2d {
        private final double x;
        private final double y;

        /** Center of the field */
        public Translation2d() {
            x = 0;
            y = 0;
        }

        /** y+ = away from red, x+ = right of red perspective */
        public Translation2d(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }

    public static class Rotation2d {
        private final double w;

        /** 0 deg */
        public Rotation2d() {
            w = 0;
        }

        /** y+ = away from red, x+ = right of red perspective */
        public Rotation2d(double deg) {
            this.w = deg;
        }

        public double getDeg() {
            return w;
        }
    }

    public static class Pose2d {
        private final Translation2d t;
        private final Rotation2d w;

        /** Center of the field facing directly to the right of red */
        public Pose2d() {
            t = new Translation2d();
            w = new Rotation2d();
        }

        public Pose2d(Translation2d translation, Rotation2d rotation) {
            this.t = translation;
            this.w = rotation;
        }

        public double getX() {
            return t.getX();
        }

        public double getY() {
            return t.getY();
        }

        public double getDeg() {
            return w.getDeg();
        }
    }

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


}
