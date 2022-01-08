package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "VisionDepoStorageBlue", group = "OpModes")
public class VisionDepoStorageBlue extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    DcMotor intake, duck;
    Servo bucket, arm;
    int location;
    OpenCvCamera webcam;
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
        webcam = robot.webcam;
    }

    @Override
    public void initAction() {
        bucket.setPosition(0.15);
        location = robot.pipeline.location;
    }

    @Override
    public void body() throws InterruptedException {

        if(location == -1 || location == 0)
        {
            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 180);

            moveBackward(0.5, 450);
            stopMotors();

            bottomLayer();

            moveForward(0.5, 750);
        }
        else if(location == 1)
        {
            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 180);

            moveBackward(0.5, 460);
            stopMotors();

            middleLayer();

            moveForward(0.5, 750);
        }
        else if(location == 2)
        {
            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 180);

            moveBackward(0.5, 500);
            stopMotors();

            topLayer();

            moveForward(0.5, 750);
        }

        moveLeft(0.5);
        Thread.sleep(2700);

        moveForward(0.2, 100);

        moveForward(0.15);
        duck.setPower(0.3);
        Thread.sleep(4000);
        stopMotors();

        duck.setPower(.65);
        Thread.sleep(1000);
        duck.setPower(0);

        moveBackward(0.5, 800);


//        if (location == 0) {
//            moveForward(0.5, 500);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.5, 450);
//            stopMotors();
//
//            bottomLayer();
//
//            moveForward(0.5, 750);
//
//            moveLeft(0.5);
//            Thread.sleep(2700);
//
//            moveForward(0.2, 100);
//
//            moveForward(0.15);
//            duck.setPower(0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            duck.setPower(.65);
//            Thread.sleep(1000);
//            duck.setPower(0);
//
//            moveBackward(0.5, 800);
//
//        }
//       else if (location == 1) {
//            moveForward(0.5, 500);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.5, 460);
//            stopMotors();
//
//            middleLayer();
//
//            moveForward(0.5, 750);
//
//            moveLeft(0.5);
//            Thread.sleep(2700);
//
//            moveForward(0.2, 100);
//
//            moveForward(0.15);
//            duck.setPower(0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            duck.setPower(.65);
//            Thread.sleep(1000);
//            duck.setPower(0);
//
//            moveBackward(0.5, 700);
//
//        }
//        else if (location == 2) {
//            moveForward(0.5, 500);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.5, 500);
//            stopMotors();
//
//            topLayer();
//
//            moveForward(0.5, 750);
//
//            moveLeft(0.5);
//            Thread.sleep(2700);
//
//            moveForward(0.2, 100);
//
//            moveForward(0.15);
//            duck.setPower(0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            duck.setPower(.65);
//            Thread.sleep(1000);
//            duck.setPower(0);
//
//            moveBackward(0.5, 700);
//
//        }
//        else if (location == -1) {
//            moveForward(0.5, 500);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.5, 500);
//            stopMotors();
//
//            topLayer();
//
//            moveForward(0.5, 750);
//
//            moveLeft(0.5);
//            Thread.sleep(2700);
//
//            moveForward(0.2, 100);
//
//            moveForward(0.15);
//            duck.setPower(0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            duck.setPower(.65);
//            Thread.sleep(1000);
//            duck.setPower(0);
//
//            moveBackward(0.5, 700);
//    }

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

    public void moveLeft(double power) {
        rf.setPower(power);
        rb.setPower(-power);
        lf.setPower(-power);
        lb.setPower(power);
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

    public void moveForward(double power) {
        rf.setPower(power);
        rb.setPower(power);
        lf.setPower(power);
        lb.setPower(power);
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

    public void moveBackward(double power) {
        rf.setPower(-power);
        rb.setPower(-power);
        lf.setPower(-power);
        lb.setPower(-power);
    }

    public void turnRight(double power, double angle) {
        double initialAngle = getIntegratedAngle();

        while(getIntegratedAngle() > initialAngle - angle + 5) {
            rf.setPower(-power);
            rb.setPower(-power);
            lf.setPower(power);
            lb.setPower(power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();

    }

    public void turnLeft(double power, double angle) {
        double initialAngle = getIntegratedAngle();

        while(getIntegratedAngle() < initialAngle + angle - 10) {
            rf.setPower(power);
            rb.setPower(power);
            lf.setPower(-power);
            lb.setPower(-power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();
    }

    public void bucketPos1()
    {
        bucket.setPosition(1);
    }


    public void bucketPos2()
    {
        bucket.setPosition(0.33);
    }

    public void topLayer() throws InterruptedException
    {
        bucket.setPosition(0.65);
        sleep(10000);
        arm.setPosition(0.6);
        sleep(10000);
        bucket.setPosition(0.9);
        sleep(10000);
        arm.setPosition(0);
        sleep(10000);
        bucket.setPosition(0.15);



    }

    public void middleLayer() throws InterruptedException {
        bucket.setPosition(0.65);
        Thread.sleep(500);
        arm.setPosition(0.8);
        Thread.sleep(500);
        bucket.setPosition(0.9);
        Thread.sleep(500);
        arm.setPosition(0);
        Thread.sleep(500);
        bucket.setPosition(0.15);
    }

    public void bottomLayer() throws InterruptedException
    {
        bucket.setPosition(0.65);
        Thread.sleep(500);
        arm.setPosition(1);
        Thread.sleep(500);
        bucket.setPosition(0.9);
        Thread.sleep(500);
        arm.setPosition(0);
        Thread.sleep(500);
        bucket.setPosition(0.15);
    }



    private double previousAngle = 0; //Outside of method
    private double integratedAngle = 0;

    /**
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     * @return The integrated heading on the interval (-inf, inf).
     */
    private double getIntegratedAngle() {
        double currentAngle = robot.imu.getAngularOrientation().firstAngle;
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle >= 180) {
            deltaAngle -= 360;
        }

        integratedAngle += deltaAngle;
        previousAngle = currentAngle;

        return integratedAngle;
    }
}
