package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Superstructure;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
@Disabled
public class MainTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Superstructure superstructure;

    @Override
    public void init() {
        superstructure = Superstructure.getInstance();
    }

    @Override
    public void loop() {
        superstructure.update();
    }
}