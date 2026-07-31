package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Superstructure;
import org.firstinspires.ftc.teamcode.Superstructure.State;

import java.util.List;

public class Auto {

    private final Superstructure superstructure;

    private List<AutoElement> elements;

    int index = 0;

    public Auto(HardwareMap hardwareMap) {
        superstructure = Superstructure.getInstance();
        superstructure.initializeHardwareMap(hardwareMap);
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
        boolean doneMoving = superstructure.moveToPose(element.targetPose);
        if (superstructure.getState() != element.targetState) {
            superstructure.setState(element.targetState);
        }

        if (superstructure.getState() == element.targetState && !superstructure.isChangingStates() && doneMoving) {
            index++;
        }

        superstructure.update();
    }

}
