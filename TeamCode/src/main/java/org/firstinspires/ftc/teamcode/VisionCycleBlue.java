package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "VisionCycleBlue", group = "OpModes")
public class VisionCycleBlue extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    DcMotor intake, duck;
    Servo bucket, arm;
    int location;
    OpenCvCamera webcam;
    ColorSensor bucketSensor;

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
        bucketSensor = robot.bucketSensor;
    }

    @Override
    public void initAction() {
        bucket.setPosition(.4);
    }

    @Override
    public void body() throws InterruptedException {
        location = robot.pipeline.location;
        if (location == 0) {
            webcam.stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5, 350);

            moveRight(0.5, 1100);

            turnLeft(0.5, 170);

            moveBackward(0.5, 75);
            stopMotors();

            bottomLayer();

        } else if (location == 1) {
            webcam.stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5, 300);

            moveRight(0.5, 1200);

            turnLeft(0.5, 170);

            moveBackward(0.5, 100);
            stopMotors();

            middleLayer();

        } else if (location == 2) {
            webcam.stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5, 300);

            moveRight(0.5, 1200);

            turnLeft(0.5, 175);

            moveBackward(0.5, 300);
            stopMotors();

            topLayer();

        }
        bucket.setPosition(.25);

        moveForward(.7,500);

        turnRight(.7,85);

        moveLeft(0.45);

        robot.safeSleep(1000);

        moveForward(.7,1300);

        moveForward(.2);

        intake.setPower(.75);

        robot.safeSleep(3000);

        intake.setPower(0);

        moveBackward(0.7, 2650);

        moveRight(0.7, 1000);

        turnLeft(0.7, 80);

        moveBackward(0.5, 100);

        topLayer();

        moveForward(.7,500);

        turnRight(.7,85);

        moveLeft(0.45);

        robot.safeSleep(1000);

        moveForward(.7,1300);

        moveForward(.2);

        intake.setPower(.75);

        robot.safeSleep(3000);

        intake.setPower(0);

        moveBackward(0.7, 2650);

        moveRight(0.7, 1000);

        turnLeft(0.7, 80);

        moveBackward(0.5, 100);

        topLayer();

        moveForward(.7,500);

        turnRight(.7,85);

        moveLeft(0.45);

        robot.safeSleep(1000);

        moveForward(.7,1300);

        moveForward(1, 500);





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

        while (rf.getCurrentPosition() < initialPos + dist) {
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

        while (rf.getCurrentPosition() > initialPos - dist) {
            rf.setPower(-power);
            rb.setPower(power);
            lf.setPower(power);
            lb.setPower(-power);
        }
        stopMotors();
    }

    public void moveForward(double power, double dist) {
        double initialPos = rf.getCurrentPosition();

        while (rf.getCurrentPosition() < initialPos + dist) {
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

        while (rf.getCurrentPosition() > initialPos - Math.abs(dist)) {
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

        while (getIntegratedAngle() > initialAngle - angle + 5) {
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

        while (getIntegratedAngle() < initialAngle + angle - 10) {
            rf.setPower(power);
            rb.setPower(power);
            lf.setPower(-power);
            lb.setPower(-power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();
    }

    public void bucketPos1() {
        bucket.setPosition(1);
    }


    public void bucketPos2() {
        bucket.setPosition(0.33);
    }

    public void topLayer() throws InterruptedException {
        bucket.setPosition(0.72);
        //bucket.setPosition(.65);
        robot.safeSleep(500);
        arm.setPosition(0.6);
        robot.safeSleep(1000);
        bucket.setPosition(1);
        robot.safeSleep(500);
        arm.setPosition(0);
        //arm.setPosition(0);
        robot.safeSleep(500);
        bucket.setPosition(0.25);
//        bucket.setPosition(0.15);

    }

    public void middleLayer() throws InterruptedException {
        bucket.setPosition(0.75);
        robot.safeSleep(500);
        arm.setPosition(0.8);
        robot.safeSleep(2000);
        bucket.setPosition(1);
        robot.safeSleep(1000);
        arm.setPosition(0);
        robot.safeSleep(500);
        bucket.setPosition(0.25);
    }

    public void bottomLayer() throws InterruptedException {
        bucket.setPosition(0.8);
        robot.safeSleep(500);
        arm.setPosition(1);
        robot.safeSleep(2000);
        bucket.setPosition(1);
        robot.safeSleep(2000);
        arm.setPosition(0);
        robot.safeSleep(500);
        bucket.setPosition(0.33);
    }

    public void IntakeCycle() throws InterruptedException {
//        if (robot.bucketSensor.alpha() < 100) {
//            moveForward(.2);
//            intake.setPower(.7);
////            robot.safeSleep(3000);
//        } else {
//            stopMotors();
//            intake.setPower(0);
        }
//    }

    private double previousAngle = 0; //Outside of method
    private double integratedAngle = 0;

    /**
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     *
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

