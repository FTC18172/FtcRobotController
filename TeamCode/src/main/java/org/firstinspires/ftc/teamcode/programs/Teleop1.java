package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.core.UpliftTele;

@TeleOp(name = "teleOp1", group = "Opmodes")
public class Teleop1 extends UpliftTele {

    UpliftRobot robot;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() throws InterruptedException {

        robot.getTest().setPower((Range.clip(gamepad1.right_stick_y, -1, 1)));

    }

    @Override
    public void exit() {

    }
}
