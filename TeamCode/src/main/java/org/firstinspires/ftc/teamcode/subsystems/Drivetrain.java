package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Helpers.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;

public class Drivetrain {

    private ChassisSpeeds chassisSpeeds;
    private Pose2d pose;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    private double lastFlPos = 0.0;
    private double lastFrPos = 0.0;
    private double lastBlPos = 0.0;
    private double lastBrPos = 0.0;

    public Drivetrain(HardwareMap hardwareMap, Pose2d pose) {
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        bl = hardwareMap.get(DcMotor.class, "leftRear");
        br = hardwareMap.get(DcMotor.class, "rightRear");
        chassisSpeeds = new ChassisSpeeds();
        this.pose = pose;

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
    }

    public void setVelocity(ChassisSpeeds velocity) {
        chassisSpeeds = velocity;
    }

    public void mecanumDrive() {
        double max;

        double axial   = chassisSpeeds.getY();
        double lateral =  chassisSpeeds.getX();
        double yaw = chassisSpeeds.getDeg() - pose.getDeg();

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

        double flPos = fl.getCurrentPosition();
        double frPos = fr.getCurrentPosition();
        double blPos = bl.getCurrentPosition();
        double brPos = br.getCurrentPosition();

        double avgFlPos = (flPos + lastFlPos) / 2.0;
        double avgFrPos = (frPos + lastFrPos) / 2.0;
        double avgBlPos = (blPos + lastBlPos) / 2.0;
        double avgBrPos = (brPos + lastBrPos) / 2.0;

        lastFlPos = flPos;
        lastFrPos = frPos;
        lastBlPos = blPos;
        lastBrPos = brPos;
    }
}