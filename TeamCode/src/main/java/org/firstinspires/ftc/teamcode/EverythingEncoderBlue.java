package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.core.UpliftAuto;

@Autonomous(name = "EverythingEncoderBlue", group = "OpModes")
public class EverythingEncoderBlue extends UpliftAuto {
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
//        moveLeft(0.5, 350);
//
//        moveForward(0.5, 500);
//
//        turnRight(0.5);
//        Thread.sleep(1350);
//
//        moveBackward(0.5, 500);
//        stopMotors();
//
//        armUp();
//
//        bucketPos1();
//        Thread.sleep(2000);
//
//        bucketPos2();
//
//        armDown();
//
//        moveForward(0.5, 800);
//
//        moveLeft(0.5, 2750);
//
//        moveForward(0.2, 100);
//
//        duck.setPower(0.3);
//        Thread.sleep(2500);
//
//        duck.setPower(.65);
//        Thread.sleep(1000);
//        stopMotors();
//
//        moveBackward(0.5, 300);
//
//        turnRight(0.5);
//        Thread.sleep(750);
//
//        moveBackward(0.5, 500);
//
//        moveForward(0.5, 1250);
//
//        turnLeft(0.5);
//        Thread.sleep(110);
//
//        moveForward(0.5, 2000);























//        moveLeft(0.5);
//        Thread.sleep(365);
//
//        moveForward(0.5);
//        Thread.sleep(500);
//
//        turnRight();
//        Thread.sleep(1550);
//
//        moveBackward(0.5);
//        Thread.sleep(390);
//        stopMotors();
//
//        armUp();
//
//        bucketPos1();
//        Thread.sleep(2000);
//
//        bucketPos2();
//
//        armDown();
//
//        moveForward(0.5);
//        Thread.sleep(500);
//
//        moveLeft(0.5);
//        Thread.sleep(2450);
//        stopMotors();
//
//        moveForward(0.1);
//        duck.setPower(0.3);
//        Thread.sleep(2500);
//
//        duck.setPower(.65);
//        Thread.sleep(1000);
//
//        moveBackward(0.5);
//        Thread.sleep(375);
//
//        turnRight(0.4);
//        Thread.sleep(600);
//
//        moveBackward(0.5);
//        Thread.sleep(500);
//
//        moveBackward(.1);
//        Thread.sleep(300);
//
//        moveForward(0.5);
//        Thread.sleep(1000);
//
//        turnLeft(0.4);
//        Thread.sleep(110);
//
//        moveForward(0.5);
//        Thread.sleep(3200);
//
//        turnLeft(0.4);
//        Thread.sleep(300);
//
//        moveForward(.3);
//        intake.setPower(.7);
//        Thread.sleep(2100);
//
//        moveBackward(.3);
//        intake.setPower(1);
//        Thread.sleep(1000);
//        stopMotors();


    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void stopMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        arm.setPower(0);
        duck.setPower(0);
    }

    public void moveLeft(double power, double dist) {
        double initialPos = rf.getCurrentPosition();

        while(rf.getCurrentPosition() < initialPos + dist) {
            rf.setPower(power);
            rb.setPower(-power);
            lf.setPower(-power);
            lb.setPower(power);
        }
        stopMotors();
    }

    public void moveRight(double power, double dist) {
        double initialPos = rf.getCurrentPosition();

        while(rf.getCurrentPosition() > initialPos - dist) {
            rf.setPower(-power);
            rb.setPower(power);
            lf.setPower(power);
            lb.setPower(-power);
        }
        stopMotors();
    }

    public void moveForward(double power, double dist) {
        double initialPos = rf.getCurrentPosition();

        while(rf.getCurrentPosition() < initialPos + dist) {
            rf.setPower(power);
            rb.setPower(power);
            lf.setPower(power);
            lb.setPower(power);
        }
        stopMotors();
    }

    public void moveBackward(double power, double dist) {
        double initialPos = rf.getCurrentPosition();

        while(rf.getCurrentPosition() > initialPos - Math.abs(dist)) {
            rf.setPower(-power);
            rb.setPower(-power);
            lf.setPower(-power);
            lb.setPower(-power);
        }
        stopMotors();
    }

    public void turnRight(double power) {
        //double initialAngle = robot.imu.getAngularOrientation().firstAngle;

        //while(robot.imu.getAngularOrientation().firstAngle < initialAngle + angle) {
            rf.setPower(-power);
            rb.setPower(-power);
            lf.setPower(power);
            lb.setPower(power);
        //    telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        //    telemetry.update();
        //}
        //stopMotors();

    }

    public void turnLeft(double power) {
        //double initialAngle = robot.imu.getAngularOrientation().firstAngle;

        //while(robot.imu.getAngularOrientation().firstAngle > initialAngle - angle) {
            rf.setPower(power);
            rb.setPower(power);
            lf.setPower(-power);
            lb.setPower(-power);
        //    telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        //}
        //stopMotors();
    }

    public void bucketPos1()
    {
            bucket.setPosition(1);
    }


    public void bucketPos2()
    {
            bucket.setPosition(0.33);
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

