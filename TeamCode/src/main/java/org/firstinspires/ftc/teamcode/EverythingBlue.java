package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.core.UpliftAuto;

@Autonomous(name = "EverythingBlue", group = "OpModes")
public class EverythingBlue extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    DcMotor intake, duck, arm;
    Servo bucket;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;
        intake = robot.intake;
        duck = robot.duck;
        arm = robot.arm;
        bucket = robot.bucket;

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        moveLeft();
        Thread.sleep(300);
        moveForward();
        Thread.sleep(500);
        turnRight();
        Thread.sleep(1600);
        moveBackward();
        Thread.sleep(350);
        stopMotors();
        armUp();
        //bucketDrop();
        //bucketDown();
        armDown();
        stopMotors();
        moveForward();
        Thread.sleep(500);
        turnRight();
        Thread.sleep(50);
        moveLeft();
        Thread.sleep(2400);
        stopMotors();
        duck.setPower(0.3);
        Thread.sleep(2500);

        duck.setPower(.65);
        Thread.sleep(1000);


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
        rf.setPower(0.5);
        rb.setPower(-0.5);
        lf.setPower(-0.5);
        lb.setPower(0.5);
    }

    public void moveRight() {
        rf.setPower(-0.5);
        rb.setPower(0.5);
        lf.setPower(0.5);
        lb.setPower(-0.5);
    }

    public void moveForward() {
        rf.setPower(0.5);
        rb.setPower(0.5);
        lf.setPower(0.5);
        lb.setPower(0.5);
    }

    public void moveBackward() {
        rf.setPower(-0.5);
        rb.setPower(-0.5);
        lf.setPower(-0.5);
        lb.setPower(-0.5);

    }

    public void turnRight() {
        rf.setPower(-0.4);
        rb.setPower(-0.4);
        lf.setPower(0.4);
        lb.setPower(0.4);

    }

    public void turnLeft() {
        rf.setPower(0.4);
        rb.setPower(0.4);
        lf.setPower(-0.4);
        lb.setPower(-0.4);
    }

    public void reverseLeft() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(-0.4);
        lb.setPower(0);
    }

    public void reverseRight() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(-0.4);
    }

    public void bucketDown()
    {
            bucket.setPosition(0.33);
    }


    public void bucketDrop()
    {
            bucket.setPosition(1);
        }

    public void armUp() throws InterruptedException
    {
            arm.setPower(0.1);
            Thread.sleep(1900);
            stopMotors();
        }

    public void armDown() throws InterruptedException
    {
            arm.setPower(-0.1);
            Thread.sleep(1800);
            stopMotors();
    }

}

