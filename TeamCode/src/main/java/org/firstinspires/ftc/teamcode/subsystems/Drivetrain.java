package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Helpers;
import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Translation2d;


public class Drivetrain {

    private Speeds robotSpeeds;
    private Pose2d pose;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    private double lastFlPos = 0.0;
    private double lastFrPos = 0.0;
    private double lastBlPos = 0.0;
    private double lastBrPos = 0.0;

    private IMU imu;

    public Drivetrain(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        bl = hardwareMap.get(DcMotor.class, "leftRear");
        br = hardwareMap.get(DcMotor.class, "rightRear");
        robotSpeeds = new Speeds();
        pose = new Pose2d();

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
    }

    public void setVelocity(Speeds velocity) {
        robotSpeeds = velocity.toRobot(pose.getDeg());
    }

    private void poseEstimate() {
        double flPos = fl.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double frPos = fr.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double blPos = bl.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double brPos = br.getCurrentPosition() * Constants.INCHES_PER_TICK;

        double dFL = (flPos + lastFlPos) / 2.0;
        double dFR = (frPos + lastFrPos) / 2.0;
        double dBL = (blPos + lastBlPos) / 2.0;
        double dBR = (brPos + lastBrPos) / 2.0;

        lastFlPos = flPos;
        lastFrPos = frPos;
        lastBlPos = blPos;
        lastBrPos = brPos;

        double dxR = (dFL + dFR + dBL + dBR) / 4.0;
        double dyR = (-dFL + dFR + dBL - dBR) / 4.0;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double avgHeading = pose.getDeg() + (heading - pose.getDeg()) / 2.0;

        pose = new Pose2d(new Translation2d(dxR * Math.cos(avgHeading / 180.0 * Math.PI) - dyR * Math.sin(avgHeading / 180.0 * Math.PI), dxR * Math.sin(avgHeading / 180.0 * Math.PI) + dyR * Math.cos(avgHeading / 180.0 * Math.PI)), new Rotation2d(heading));
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

    public void update() {
        poseEstimate();
        mecanumDrive();
    }
}