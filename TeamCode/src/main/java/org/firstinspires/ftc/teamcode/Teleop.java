package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftTele;

import java.awt.font.NumericShaper;
@TeleOp(name = "teleOp", group = "Opmodes")
public class Teleop extends UpliftTele {
    DcMotor lf, rf, lb, rb;
    @Override
    public void initHardware() {
        UpliftRobot robot = new UpliftRobot(this);
        lf = robot.leftFront;
        lb = robot.leftBack;
        rf = robot.rightFront;
        rb = robot.rightBack;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() {
        lf.setPower(Range.clip(-gamepad1.left_stick_y,-1,1));
        lb.setPower(Range.clip(-gamepad1.left_stick_y,-1,1));
        rf.setPower(Range.clip(-gamepad1.right_stick_y,-1,1));
        rb.setPower(Range.clip(-gamepad1.right_stick_y,-1,1));
    }

    @Override
    public void exit() {

    }
}
