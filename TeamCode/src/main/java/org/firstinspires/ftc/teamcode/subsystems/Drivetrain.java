package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Helpers.VisionMeasurement;
import org.firstinspires.ftc.teamcode.utils.Helpers.PoseMeasurement;
import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Translation2d;
import org.firstinspires.ftc.teamcode.Superstructure.DriveState;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {

    private Speeds robotSpeeds;
    private Pose2d pose;
    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;

    private double lastFlPos = 0.0;
    private double lastFrPos = 0.0;
    private double lastBlPos = 0.0;
    private double lastBrPos = 0.0;

    private final List<PoseMeasurement> poseHistory;

    private final IMU imu;

    public Drivetrain(HardwareMap hardwareMap, Pose2d initialPose) {
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        bl = hardwareMap.get(DcMotor.class, "leftRear");
        br = hardwareMap.get(DcMotor.class, "rightRear");
        robotSpeeds = new Speeds();
        pose = initialPose;

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // set directions of motors
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        poseHistory = new ArrayList<>();
    }

    public void setVelocity(Speeds velocity) {
        robotSpeeds = velocity.toRobot(pose.getDeg());
    }

    private void poseEstimate() {
        double flPos = fl.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double frPos = fr.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double blPos = bl.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double brPos = br.getCurrentPosition() * Constants.INCHES_PER_TICK;

        double dFL = flPos - lastFlPos;
        double dFR = frPos - lastFrPos;
        double dBL = blPos - lastBlPos;
        double dBR = brPos - lastBrPos;

        lastFlPos = flPos;
        lastFrPos = frPos;
        lastBlPos = blPos;
        lastBrPos = brPos;

        double dxR = (dFL + dFR + dBL + dBR) / 4.0;
        double dyR = (-dFL + dFR + dBL - dBR) / 4.0;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double avgHeading = pose.getDeg() + (heading - pose.getDeg()) / 2.0;

        // encoder estimated position
        double estimatedX = dxR * Math.cos(avgHeading / 180.0 * Math.PI) - dyR * Math.sin(avgHeading / 180.0 * Math.PI);
        double estimatedY = dxR * Math.sin(avgHeading / 180.0 * Math.PI) + dyR * Math.cos(avgHeading / 180.0 * Math.PI);

        poseHistory.add(new PoseMeasurement(new Pose2d(pose.getX() + estimatedX, pose.getY() + estimatedY, heading), System.currentTimeMillis()));

        while (poseHistory.size() > Constants.MAX_POSE_HISTORY) {
            poseHistory.remove(0);
        }

        pose = poseHistory.get(poseHistory.size() - 1).pose;

    }

    public void addVisionPose(VisionMeasurement visionMeasurement) {
        if (poseHistory.isEmpty()) return;

        // find the closest pose to the timestamp from the vision measurement
        PoseMeasurement closest = poseHistory.get(0);
        double minDiff = Math.abs(visionMeasurement.timestamp - closest.timestamp);
        int index = 0;
        for (int i = 1; i < poseHistory.size(); i++) {
            PoseMeasurement candidate = poseHistory.get(i);
            double diff = Math.abs(visionMeasurement.timestamp - candidate.timestamp);
            if (diff < minDiff) {
                minDiff = diff;
                closest = candidate;
                index = i;
            }
        }

        double offsetX = (visionMeasurement.pose.getX() - closest.pose.getX()) * visionMeasurement.confidence;
        double offsetY = (visionMeasurement.pose.getY() - closest.pose.getY()) * visionMeasurement.confidence;

        for (int i = index; i < poseHistory.size(); i++) {
            poseHistory.get(i).pose = new Pose2d(poseHistory.get(i).pose.getX() + offsetX, poseHistory.get(i).pose.getY() + offsetY, poseHistory.get(i).pose.getDeg());
        }

        pose = poseHistory.get(poseHistory.size() - 1).pose;

    }

    private void mecanumDrive() {
        double max;

        double axial   = robotSpeeds.getY();
        double lateral =  robotSpeeds.getX();
        double yaw = robotSpeeds.getDeg() - pose.getDeg();

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double flPower = axial + lateral + yaw;
        double frPower = axial - lateral - yaw;
        double blPower   = axial - lateral + yaw;
        double brPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower   /= max;
            brPower  /= max;
        }

        // Send calculated power to wheels
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }


    public Pose2d update(DriveState robotState) {
        poseEstimate();
        mecanumDrive();
        return pose;
    }
}
