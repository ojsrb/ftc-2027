package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Helpers.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.Helpers.Translation2d;

public class Vision {
    public Pose2d getVisionPose() {
        return new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
    }
}
