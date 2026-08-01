package org.firstinspires.ftc.teamcode;

public class Constants {
    // ------------------
    // ----- TELEOP -----
    // ------------------
    public static final double JOYSTICK_POW = 2.0; // exponential input curve
    public static final double ROBOT_MAX_SPEED = 1.0; // maximum feet per second
    public static final double ROBOT_SPEED_PERCENT = 1.0; // change when testing etc.

    // ------------------------
    // ----- DRIVE MOTORS -----
    // ------------------------

    public static final double MotorP = 1.0;
    public static final double MotorI = 0.0;
    public static final double MotorD = 0.0;



    // ---------------------
    // ----- DRIVE PID -----
    // ---------------------

    // translation
    public static final double tP = 0.0;
    public static final double tI = 0.0;
    public static final double tD = 0.0;

    // rotation
    public static final double wP = 0.0;
    public static final double wI = 0.0;
    public static final double wD = 0.0;

    // finished tolerances
    public static final double tTolerance = 0.1;
    public static final double wTolerance = 1.0;



    // ---------------------------
    // ----- POSE ESTIMATION -----
    // ---------------------------

    public static final int MAX_POSE_HISTORY = 64;
    public static final double DETECTION_STALE_THRESH_MS = 500.0; // maximum latency before vision detections are rejected
    public static final double DETECTION_CONFIDENCE_THRESH = 0.2; // minimum required confidence for a detection to be accepted
}
