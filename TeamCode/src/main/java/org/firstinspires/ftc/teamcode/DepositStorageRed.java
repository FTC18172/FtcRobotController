package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;

@Autonomous(name = "DepositStorageRed", group = "OpModes")
public class DepositStorageRed extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    DcMotor intake, duck;
    Servo bucket, arm;

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
        moveLeft(0.5);
        Thread.sleep(300);

        moveForward(0.5);
        Thread.sleep(1050);

        turnLeft(0.4);
        Thread.sleep(1000);

        moveBackward(0.5);
        Thread.sleep(300);
        stopMotors();

        armUp();

        bucketPos1();
        Thread.sleep(2000);

        bucketPos2();
        Thread.sleep(500);

        armDown();

        moveForward(0.5);
        Thread.sleep(300);

        turnLeft(0.4);
        Thread.sleep(550);

        moveRight(0.5);
        Thread.sleep(1800);

        moveLeft(0.3);
        Thread.sleep(550);

        moveForward(0.3);
        Thread.sleep(1200);

        turnRight(0.3);
        Thread.sleep(550);

        moveForward(.2);
        Thread.sleep(200);

        moveForward(0.1);
        duck.setPower(-0.3);
        Thread.sleep(3000);

        duck.setPower(-.65);
        Thread.sleep(1000);

        turnLeft(.3);
        Thread.sleep(500);

        moveBackward(0.5);
        Thread.sleep(800);
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
//        arm.setPower(0.1);
//        Thread.sleep(2100);
//        stopMotors();
    }

    public void armDown() throws InterruptedException
    {
//        arm.setPower(-0.1);
//        Thread.sleep(1800);
//        stopMotors();
    }

}
