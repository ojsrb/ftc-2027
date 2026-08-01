package org.firstinspires.ftc.teamcode.auto;

import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Superstructure.State;

public class AutoElement {

    final Pose2d targetPose;
    final State targetState;

    public AutoElement(Pose2d pose, State state) {
        targetPose = pose;
        targetState = state;
    }
}
