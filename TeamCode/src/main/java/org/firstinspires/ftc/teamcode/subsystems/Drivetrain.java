package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.Helpers.PoseMeasurement;
import org.firstinspires.ftc.teamcode.utils.Helpers.VisionMeasurement;
import org.firstinspires.ftc.teamcode.utils.Units;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {

    // pose uses inches and degrees
    private Pose2d pose;
    private final Motor fl;
    private final Motor fr;
    private final Motor bl;
    private final Motor br;

    private final List<PoseMeasurement> poseHistory;

    private final IMU imu;

    // wheel locations in meters
    final Translation2d m_frontLeftLocation =
            new Translation2d(Units.in2m(10), Units.in2m(10));
    final Translation2d m_frontRightLocation =
            new Translation2d(Units.in2m(10), -Units.in2m(10));
    final Translation2d m_backLeftLocation =
            new Translation2d(Units.in2m(10), Units.in2m(10));
    final Translation2d m_backRightLocation =
            new Translation2d(Units.in2m(10), -Units.in2m(10));

    MecanumDriveKinematics kinematics;
    MecanumDriveWheelSpeeds wheelSpeeds;
    final MecanumDriveOdometry odometry;

    final MecanumDrive drive;

    public Drivetrain(HardwareMap hardwareMap, Pose2d initialPose) {
        wheelSpeeds = new MecanumDriveWheelSpeeds();
        pose = initialPose;
        odometry = new MecanumDriveOdometry(
                kinematics, pose.getRotation(), pose
        );

        fl = new Motor(hardwareMap, "leftFront");
        fr = new Motor(hardwareMap, "rightFront");
        bl = new Motor(hardwareMap, "leftRear");
        br = new Motor(hardwareMap, "rightRear");

        fl.setRunMode(Motor.RunMode.VelocityControl);
        fr.setRunMode(Motor.RunMode.VelocityControl);
        bl.setRunMode(Motor.RunMode.VelocityControl);
        br.setRunMode(Motor.RunMode.VelocityControl);

        fl.setVeloCoefficients(Constants.MotorP, Constants.MotorI, Constants.MotorD);
        fr.setVeloCoefficients(Constants.MotorP, Constants.MotorI, Constants.MotorD);
        bl.setVeloCoefficients(Constants.MotorP, Constants.MotorI, Constants.MotorD);
        br.setVeloCoefficients(Constants.MotorP, Constants.MotorI, Constants.MotorD);

        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // set directions of motors
        fl.setInverted(true);
        fr.setInverted(false);
        bl.setInverted(true);
        br.setInverted(false);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        poseHistory = new ArrayList<>();

        kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation,
                m_frontRightLocation,
                m_backLeftLocation,
                m_backRightLocation
        );

        drive = new MecanumDrive(
                fl, fr,
                bl, br
        );
    }

    public void setVelocity(ChassisSpeeds velocityInchesDegrees) {
        wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                Units.in2m(velocityInchesDegrees.vxMetersPerSecond),
                Units.in2m(velocityInchesDegrees.vyMetersPerSecond),
                Units.deg2rad(velocityInchesDegrees.omegaRadiansPerSecond),
                Rotation2d.fromDegrees(pose.getHeading())
        ));
    }

    private void poseEstimate() {

        // update pose with odometry
        MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds(
                fl.encoder.getRate(), fr.encoder.getRate(),
                bl.encoder.getRate(), br.encoder.getRate()
        );

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        Rotation2d avgHeading = Rotation2d.fromDegrees(pose.getHeading() + (heading - pose.getHeading()) / 2.0);

        Pose2d odometryPose = odometry.updateWithTime(Units.ms2sec(System.currentTimeMillis()), avgHeading, speeds);

        poseHistory.add(new PoseMeasurement(Units.m2in(odometryPose), System.currentTimeMillis()));

        while (poseHistory.size() > Constants.MAX_POSE_HISTORY) {
            poseHistory.remove(0);
        }

        pose = poseHistory.get(poseHistory.size() - 1).pose;

    }

    public void addVisionPose(VisionMeasurement visionMeasurement) {
        if (poseHistory.isEmpty()) return;
        if (visionMeasurement.confidence < Constants.DETECTION_CONFIDENCE_THRESH) return;

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

        if (minDiff >= Constants.DETECTION_STALE_THRESH_MS) {
            return;
        }

        double offsetX = (visionMeasurement.pose.getX() - closest.pose.getX()) * visionMeasurement.confidence;
        double offsetY = (visionMeasurement.pose.getY() - closest.pose.getY()) * visionMeasurement.confidence;

        for (int i = index; i < poseHistory.size(); i++) {
            poseHistory.get(i).pose = new Pose2d(poseHistory.get(i).pose.getX() + offsetX, poseHistory.get(i).pose.getY() + offsetY, poseHistory.get(i).pose.getHeading());
        }

        pose = poseHistory.get(poseHistory.size() - 1).pose;

    }

    private void mecanumDrive() {
        fl.set(wheelSpeeds.frontLeftMetersPerSecond);
        fr.set(wheelSpeeds.frontRightMetersPerSecond);
        bl.set(wheelSpeeds.rearLeftMetersPerSecond);
        br.set(wheelSpeeds.rearRightMetersPerSecond);

    }


    public Pose2d update() {
        poseEstimate();
        mecanumDrive();
        return pose;
    }
}
