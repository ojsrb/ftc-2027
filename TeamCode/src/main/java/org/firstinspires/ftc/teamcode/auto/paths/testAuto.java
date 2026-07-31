package org.firstinspires.ftc.teamcode.auto.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.auto.Auto;

@Autonomous(name = "testAuto")
@Disabled
public class testAuto extends LinearOpMode{

    private Auto auto;

    @Override public void runOpMode() {
        auto.add(new Pose2d(0, 0, 0));
        auto.add(new Pose2d(0, 20, 0));
        auto.add(new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            auto.update();
        }

    }


}
