package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.VisionMeasurement;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.List;

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

    private final boolean changingStates = false;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController wController;
    private Superstructure() {
        state = State.IDLE;

        xController = new PIDController(Constants.tkP, Constants.tkI, Constants.tkD);
        yController = new PIDController(Constants.tkP, Constants.tkI, Constants.tkD);
        wController = new PIDController(Constants.wkP, Constants.wkI, Constants.wkD);

        xController.setTolerance(Constants.tTolerance);
        yController.setTolerance(Constants.tTolerance);
        wController.setTolerance(Constants.wTolerance);

        wController.enableContinuousInput(-180.0, 180.0);
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
        List<VisionMeasurement> visionMeasurements = vision.getMeasurements();
        for (VisionMeasurement measurement : visionMeasurements) {
            drivetrain.addVisionPose(measurement);
        }
        pose = drivetrain.update(driveState);
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

    public boolean moveToPose(Pose2d targetPose) {
        double xControl = xController.calculate(pose.getX(), targetPose.getX());
        double yControl = yController.calculate(pose.getY(), targetPose.getY());
        double wControl = wController.calculate(pose.getHeading(), targetPose.getHeading());

        Speeds newSpeeds = new Speeds(xControl, yControl, wControl);
        setDrivetrainSpeeds(newSpeeds);

        if (xController.atSetpoint() && yController.atSetpoint() && wController.atSetpoint()) {
            setDrivetrainSpeeds(new Speeds(0, 0, 0));
            return true;
        } else {
            return false;
        }

    }

}
