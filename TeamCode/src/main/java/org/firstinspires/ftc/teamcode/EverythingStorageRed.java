package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;

@Autonomous(name = "EverythingStorageRed", group = "OpModes")
public class EverythingStorageRed extends UpliftAuto {
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
        moveRight(.5);
        Thread.sleep(500);

        moveForward(.5);
        Thread.sleep(1000);

        turnLeft(.4);
        Thread.sleep(500);
        stopMotors();

        moveForward(.3);
        Thread.sleep(700);
        stopMotors();

        duck.setPower(-0.3);
        Thread.sleep(2500);

        duck.setPower(-.65);
        Thread.sleep(1000);

        turnLeft(.4);
        Thread.sleep(300);

        moveBackward(.5);
        Thread.sleep(1075);

        turnLeft(.4);
        Thread.sleep(800);

        moveBackward(0.25);
        Thread.sleep(1250);

        moveForward(.5);
        Thread.sleep(400);

        turnLeft(.4);
        Thread.sleep(1500);

        moveBackward(.4);
        Thread.sleep(1000);
        stopMotors();

        armUp();

        bucketPos1();
        Thread.sleep(2000);

        bucketPos2();
        Thread.sleep(500);

        armDown();

        moveForward(.4);
        Thread.sleep(1300);

        moveLeft(0.3);
        Thread.sleep(950);
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

    public void moveLeft(double power) {

        rf.setPower(power);
        rb.setPower(-power);
        lf.setPower(-power);
        lb.setPower(power);
    }

    public void moveRight(double power) {

        rf.setPower(-power);
        rb.setPower(power);
        lf.setPower(power);
        lb.setPower(-power);
    }

    public void moveForward(double power) {

        rf.setPower(power);
        rb.setPower(power);
        lf.setPower(power);
        lb.setPower(power);
    }

    public void moveBackward(double power) {

        rf.setPower(-power);
        rb.setPower(-power);
        lf.setPower(-power);
        lb.setPower(-power);
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
