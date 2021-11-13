package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;
@Autonomous(name = "DuckAutoBlue", group = "OpModes")
public class DuckAutoBlue extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    DcMotor duck;



    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;
        duck = robot.duck;

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {

        moveLeft();
        Thread.sleep(500);
        moveForward();
        Thread.sleep(1000);
        turnRight();
        Thread.sleep(850);
        duck.setPower(0.3);
        Thread.sleep(3000);
        duck.setPower(.65);
        reverseLeft();
        Thread.sleep(1000);
        moveLeft();
        Thread.sleep(1000);
        stopMotors();
    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void stopMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    public void moveLeft() {
        rf.setPower(-0.5);
        rb.setPower(-0.5);
        lf.setPower(-0.5);
        lb.setPower(-0.5);
    }

    public void moveRight() {
        rf.setPower(0.5);
        rb.setPower(0.5);
        lf.setPower(0.5);
        lb.setPower(0.5);
    }

    public void moveForward() {
        rf.setPower(-0.5);
        rb.setPower(0.5);
        lf.setPower(0.5);
        lb.setPower(-0.5);
    }

    public void moveBackward() {
        rf.setPower(0.5);
        rb.setPower(-0.5);
        lf.setPower(-0.5);
        lb.setPower(0.5);

    }

    public void turnRight() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0.4);
        lb.setPower(0);

    }

    public void reverseLeft() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(-0.4);
        lb.setPower(0);
    }

    public void spin(double speed) {
        speed = Range.clip(speed, -1, 1);
        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(-speed);
        rb.setPower(-speed);
    }
    public void reverseRight() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(-0.4);
    }

}


