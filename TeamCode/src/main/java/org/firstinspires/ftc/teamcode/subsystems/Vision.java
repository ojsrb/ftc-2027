package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Translation2d;

public class Vision {

    public Vision(HardwareMap hardwareMap) {
        // Initialize vision system here
    }

    public Pose2d estimatePose() {
        return new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
    }
}
