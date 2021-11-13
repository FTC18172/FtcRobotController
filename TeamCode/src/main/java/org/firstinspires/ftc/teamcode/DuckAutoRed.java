package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;

@Autonomous(name = "DuckAutoRed", group = "OpModes")
public class DuckAutoRed extends UpliftAuto {
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

        moveRight();
        Thread.sleep(500);

        moveForward();
        Thread.sleep(1000);

        turnLeft();
        Thread.sleep(500);
        stopMotors();

        moveForward();
        Thread.sleep(280);
        stopMotors();

        duck.setPower(-0.3);
        Thread.sleep(2500);

        duck.setPower(-.65);
        Thread.sleep(1000);

        turnLeft();
        Thread.sleep(400);
        stopMotors();

        moveBackward();
        Thread.sleep(900);
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

}
