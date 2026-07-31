package org.firstinspires.ftc.teamcode;

public class Constants {
    // ------------------
    // ----- TELEOP -----
    // ------------------
    public static final double JOYSTICK_POW = 2.0;
    public static final double SPEED = 1.0;

    // -------------------
    // ----- ENCODER -----
    // -------------------

    public static final double INCHES_PER_TICK = 1.0;



    // ---------------------
    // ----- DRIVE PID -----
    // ---------------------

    // translation
    public static final double tkP = 0.0;
    public static final double tkI = 0.0;
    public static final double tkD = 0.0;

    // rotation
    public static final double wkP = 0.0;
    public static final double wkI = 0.0;
    public static final double wkD = 0.0;

    // finished tolerances
    public static final double tTolerance = 0.1;
    public static final double wTolerance = 1.0;



    // ---------------------------
    // ----- POSE ESTIMATION -----
    // ---------------------------

    public static final int MAX_POSE_HISTORY= 64;
}
