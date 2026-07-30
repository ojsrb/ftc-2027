package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.Superstructure.State;

public class AutoElement {

    Pose2d targetPose;
    State targetState;

    public AutoElement(Pose2d pose, State state) {
        targetPose = pose;
        targetState = state;
    }
}
