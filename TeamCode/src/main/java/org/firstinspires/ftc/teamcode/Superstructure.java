package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

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

        ChassisSpeeds newSpeeds = new ChassisSpeeds(xControl, yControl, wControl);
        setDrivetrainSpeeds(newSpeeds);

        if (xController.atSetpoint() && yController.atSetpoint() && wController.atSetpoint()) {
            setDrivetrainSpeeds(new ChassisSpeeds(0, 0, 0));
            return true;
        } else {
            return false;
        }

    }

}
