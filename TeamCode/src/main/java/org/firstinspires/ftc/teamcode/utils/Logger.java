package org.firstinspires.ftc.teamcode.utils;

public class Logger {

    public enum Status {
        INITIALIZED,
    }

    public void setStatus(Status status) {
        String message;

        switch (status) {
            case INITIALIZED:
                message = "Initialized";
                break;
            default:
                break;
        }
    }

}
