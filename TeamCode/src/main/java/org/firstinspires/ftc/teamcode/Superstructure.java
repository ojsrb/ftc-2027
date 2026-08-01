package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.Helpers.VisionMeasurement;

import java.util.List;

public class Superstructure {

    public enum State {
        IDLE,
    }

    private Pose2d pose;
    private Vision vision;
    private Drivetrain drivetrain;

    private State state;

    private final boolean changingStates = false;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController wController;
    private Superstructure() {
        state = State.IDLE;

        xController = new PIDController(Constants.tP, Constants.tI, Constants.tD);
        yController = new PIDController(Constants.tP, Constants.tI, Constants.tD);
        wController = new PIDController(Constants.wP, Constants.wI, Constants.wD);

        xController.setTolerance(Constants.tTolerance);
        yController.setTolerance(Constants.tTolerance);
        wController.setTolerance(Constants.wTolerance);
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
        pose = drivetrain.update();
    }

    public void setDrivetrainSpeeds(ChassisSpeeds speeds) {
        drivetrain.setVelocity(speeds);
    }

    public void setState(State newState) {
        if (!changingStates) {
            state = newState;
        }
    }

    public State getState() {
        return state;
    }

    public boolean isChangingStates() {
        return changingStates;
    }

    public boolean moveToPose(Pose2d targetPose) {
        double xControl = xController.calculate(pose.getX(), targetPose.getX());
        double yControl = yController.calculate(pose.getY(), targetPose.getY());

        double wrappedError = targetPose.getHeading() - pose.getHeading();
        while (wrappedError > 180)  wrappedError -= 360;
        while (wrappedError < -180) wrappedError += 360;

        wController.setSetPoint(0);

        double wControl = wController.calculate(wrappedError);

        ChassisSpeeds newSpeeds = new ChassisSpeeds(xControl, yControl, wControl);
        setDrivetrainSpeeds(newSpeeds);

        if (xController.atSetPoint() && yController.atSetPoint() && wController.atSetPoint()) {
            setDrivetrainSpeeds(new ChassisSpeeds(0, 0, 0));
            return true;
        } else {
            return false;
        }

    }

}
