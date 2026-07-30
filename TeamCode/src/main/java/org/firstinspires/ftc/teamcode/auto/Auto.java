package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Superstructure.State;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;

import java.util.List;

public class Auto {

    Follower follower;

    List<AutoElement> elements;

    int index = 0;

    public Auto(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
    }

    public void add(State targetState) {
        if (elements.isEmpty()) {
            elements.add(new AutoElement(new Pose2d(), targetState));
        } else {
            elements.add(new AutoElement(elements.get(elements.size() - 1).targetPose, targetState));
        }
    }

    public void add(Pose2d targetPose) {
        if (elements.isEmpty()) {
            elements.add(new AutoElement(targetPose, State.IDLE));
        } else {
            elements.add(new AutoElement(targetPose, elements.get(elements.size() - 1).targetState));
        }
    }

    public void add(Pose2d targetPose, State targetState) {
        elements.add(new AutoElement(targetPose, targetState));
    }

    public void update() {
        AutoElement element = elements.get(index);
        boolean doneMoving = follower.moveToPose(element.targetPose);
        if (follower.superstructure.getState() != element.targetState) {
            follower.superstructure.setState(element.targetState);
        }

        if (follower.superstructure.getState() == element.targetState && !follower.superstructure.isChangingStates() && doneMoving) {
            index++;
        }
    }

}
