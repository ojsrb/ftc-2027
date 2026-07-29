package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Superstructure {

    public enum State {
        IDLE,
        MOVING
    }

    private Pose2d pose;
    private Vision vision;
    private Drivetrain drivetrain;

    private State state;

    private Superstructure() {
        vision = new Vision();
        pose = vision.getVisionPose();
        state = State.IDLE;
        drivetrain = new Drivetrain(hardwareMap);
    }

    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public void update() {
        drivetrain.update();
    }

    public void drivetrainSpeeds(Speeds speeds) {
        drivetrain.setVelocity(speeds);
    }

    public void setState(State newState) {
        state = newState;
    }

    public State getState() {
        return state;
    }

    public Pose2d getPose() {
        return pose;
    }

}
