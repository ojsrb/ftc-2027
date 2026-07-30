package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Superstructure {

    public enum State {
        IDLE,
        INTAKE,
        SCORE,
        DEPOSIT,
        TRAVEL
    }

    public enum DriveState {
        TELEOP,
        AUTO,
        ALIGN
    }

    private Pose2d pose;
    private Vision vision;
    private Drivetrain drivetrain;

    private DriveState driveState;

    private State state;

    private boolean changingStates = false;

    private Superstructure() {
        state = State.IDLE;
    }

    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public void initializeHardwareMap(HardwareMap hardwareMap) {
        vision = new Vision(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, pose);
    }

    public void update() {
        pose = drivetrain.update(driveState, vision.estimatePose());
    }

    public void setDrivetrainSpeeds(Speeds speeds) {
        drivetrain.setVelocity(speeds);
    }

    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    public void setState(State newState) {
        if (!changingStates) {
            state = newState;
        }
    }

    public State getState() {
        return state;
    }

    public Pose2d getPose() {
        return pose;
    }

    public boolean isChangingStates() {
        return changingStates;
    }

}
