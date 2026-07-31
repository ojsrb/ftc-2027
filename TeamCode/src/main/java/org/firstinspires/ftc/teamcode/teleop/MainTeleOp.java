package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Superstructure;
import org.firstinspires.ftc.teamcode.utils.Helpers;
import org.firstinspires.ftc.teamcode.utils.Helpers.Speeds;

@TeleOp(name = "MainTeleOp", group = "TeleOp")
@Disabled
public class MainTeleOp extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    private GamepadEx driverGamepad;

    private Superstructure superstructure;

    @Override
    public void init() {
        superstructure = Superstructure.getInstance();
        driverGamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        Speeds driveSpeeds = new Speeds(Math.pow(driverGamepad.getLeftX(), Constants.JOYSTICK_POW) * Constants.SPEED, Math.pow(driverGamepad.getLeftY(), Constants.JOYSTICK_POW) * Constants.SPEED, Math.pow(driverGamepad.getRightX(), Constants.JOYSTICK_POW));
        superstructure.setDrivetrainSpeeds(driveSpeeds);
        superstructure.update();
    }
}