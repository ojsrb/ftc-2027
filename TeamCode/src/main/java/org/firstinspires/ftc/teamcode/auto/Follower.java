package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Superstructure;
import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Follower {

    public Superstructure superstructure;

    private Constants constants;

    PIDController xController;
    PIDController yController;
    PIDController wController;

    public Follower(HardwareMap hardwareMap) {
        superstructure = Superstructure.getInstance();
        superstructure.initializeHardwareMap(hardwareMap);
        constants = new Constants();

        xController = new PIDController(constants.tkP, constants.tkI, constants.tkD);
        yController = new PIDController(constants.tkP, constants.tkI, constants.tkD);
        wController = new PIDController(constants.wkP, constants.wkI, constants.wkD);

        xController.setTolerance(constants.tTolerance);
        yController.setTolerance(constants.tTolerance);
        wController.setTolerance(constants.wTolerance);

        wController.enableContinuousInput(-180.0, 180.0);

    }

    public boolean moveToPose(Pose2d targetPose) {
        Pose2d robotPose = superstructure.getPose();

        double xcontrol = xController.calculate(robotPose.getX(), targetPose.getX());
        double ycontrol = yController.calculate(robotPose.getY(), targetPose.getY());
        double wcontrol = wController.calculate(robotPose.getDeg(), targetPose.getDeg());

        Speeds newSpeeds = new Speeds(xcontrol, ycontrol, wcontrol);
        superstructure.setDrivetrainSpeeds(newSpeeds);

        if (xController.atSetpoint() && yController.atSetpoint() && wController.atSetpoint()) {
            superstructure.setDrivetrainSpeeds(new Speeds(0, 0, 0));
            return true;
        } else {
            return false;
        }

    }

}
