package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.core.UpliftRobot;

import static java.lang.Math.abs;

@Autonomous(name = "ParkAuto", group = "OpModes")
public class ParkAuto extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {

        moveForward();
        Thread.sleep(1200);
        stopMotors();
    }

    @Override
    public void exit() throws InterruptedException {

    }

    public void stopMotors()
    {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    public void moveLeft()
    {
        rf.setPower(0.5);
        rb.setPower(-0.5);
        lf.setPower(-0.5);
        lb.setPower(0.5);
    }

    public void moveRight()
    {
        rf.setPower(-0.5);
        rb.setPower(0.5);
        lf.setPower(0.5);
        lb.setPower(-0.5);
    }

    public void moveForward()
    {
        rf.setPower(0.5);
        rb.setPower(0.5);
        lf.setPower(0.5);
        lb.setPower(0.5);
    }

    public void moveBackward()
    {
        rf.setPower(-0.5);
        rb.setPower(-0.5);
        lf.setPower(-0.5);
        lb.setPower(-0.5);

    }

    public void practiceMoveForward()
    {
        
    }


}