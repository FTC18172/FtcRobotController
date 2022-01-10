package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "VisionEverythingBlue", group = "OpModes")
public class VisionEverythingBlue extends UpliftAuto {
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
        bucket.setPosition(0.3);
    }

    @Override
    public void body() throws InterruptedException {
        location = robot.pipeline.location;
        if(location == 0)
        {
            webcam.stopRecordingPipeline();
            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 180);

            moveBackward(0.5, 125);
            stopMotors();

            bottomLayer();

            moveForward(0.5, 150);
        }
        else if(location == 1)
        {
            webcam.stopRecordingPipeline();

            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 180);

            moveBackward(0.5, 240);
            stopMotors();

            middleLayer();

            moveForward(0.5, 400);
        }
        else if(location == 2 || location == -1 )
        {
            webcam.stopRecordingPipeline();
            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 180);

            moveBackward(0.5, 500);
            stopMotors();

            topLayer();

            moveForward(0.5, 750);     }

        bucket.setPosition(0.3);

        moveLeft(0.5);
        robot.safeSleep(2700);

        moveForward(0.2, 100);

        moveForward(0.15);
        duck.setPower(0.3);
        robot.safeSleep(4000);
        stopMotors();

        duck.setPower(.65);
        robot.safeSleep(1000);
        duck.setPower(0);

        moveBackward(0.5, 300);

        turnRight(0.5, 80);

        moveBackward(0.5);
        Thread.sleep(300);

        moveBackward(0.2);
        Thread.sleep(800);

        moveForward(0.5, 1250);

        turnLeft(0.5, 11);

        moveForward(0.5, 4000);

        moveForward(.2, 500);

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
        bucket.setPosition(0.62);
        //bucket.setPosition(.65);
        robot.safeSleep(500);
        arm.setPosition(0.6);
        robot.safeSleep(1000);
        bucket.setPosition(0.9);
        robot.safeSleep(500);
        arm.setPosition(0);
        //arm.setPosition(0);
        robot.safeSleep(500);
        bucket.setPosition(0.25);
//        bucket.setPosition(0.15);

    }

    public void middleLayer() throws InterruptedException {
        bucket.setPosition(0.65);
        robot.safeSleep(500);
        arm.setPosition(0.8);
        robot.safeSleep(1000);
        bucket.setPosition(0.9);
        robot.safeSleep(500);
        arm.setPosition(0);
        robot.safeSleep(500);
        bucket.setPosition(0.25);
    }

    public void bottomLayer() throws InterruptedException
    {
        bucket.setPosition(0.65);
        robot.safeSleep(500);
        arm.setPosition(1);
        robot.safeSleep(1000);
        bucket.setPosition(0.9);
        robot.safeSleep(500);
        arm.setPosition(0);
        robot.safeSleep(500);
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
