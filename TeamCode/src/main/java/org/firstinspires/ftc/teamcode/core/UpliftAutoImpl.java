package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;

public class UpliftAutoImpl extends UpliftAuto {
    private double previousAngle = 0;
    private double integratedAngle = 0;

    public UpliftRobot robot;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {
//        robot.getBucket().setPosition(0.5);
    }


    @Override
    public void body() throws InterruptedException {

    }

    @Override
    public void exit() throws InterruptedException {

    }
//
//    public void stopMotors() {
//        robot.getLeftFront().setPower(0);
//        robot.getLeftBack().setPower(0);
//        robot.getRightFront().setPower(0);
//        robot.getRightBack().setPower(0);
//    }
//
//    public void topLayer() throws InterruptedException {
//        robot.getBucket().setPosition(0.72);
//        //bucket.setPosition(.65);
//        robot.safeSleep(500);
//        robot.getArm().setPosition(0.6);
//        robot.safeSleep(1000);
//        robot.getBucket().setPosition(1);
//        robot.safeSleep(500);
//        robot.getArm().setPosition(0);
//        //arm.setPosition(0);
//        robot.safeSleep(500);
//        robot.getBucket().setPosition(0.25);
//        //        bucket.setPosition(0.15);
//    }
//
//    public void middleLayer() throws InterruptedException {
//        robot.getBucket().setPosition(0.75);
//        robot.safeSleep(500);
//        robot.getArm().setPosition(0.8);
//        robot.safeSleep(2000);
//        robot.getBucket().setPosition(1);
//        robot.safeSleep(1000);
//        robot.getArm().setPosition(0);
//        robot.safeSleep(500);
//        robot.getBucket().setPosition(0.25);
//    }
//
//    public void bottomLayer() throws InterruptedException {
//        robot.getBucket().setPosition(0.75);
//        robot.safeSleep(500);
//        robot.getArm().setPosition(1);
//        robot.safeSleep(2000);
//        robot.getBucket().setPosition(1);
//        robot.safeSleep(2000);
//        robot.getArm().setPosition(0);
//        robot.safeSleep(500);
//        robot.getBucket().setPosition(0.33);
//    }
//
//    public void moveLeft(double power, double dist) {
//        double initialPos = robot.getRightFront().getCurrentPosition();
//
//        while (robot.getRightFront().getCurrentPosition() < initialPos + dist) {
//            robot.getRightFront().setPower(power);
//            robot.getRightBack().setPower(-power);
//            robot.getLeftFront().setPower(-power);
//            robot.getLeftBack().setPower(power);
//        }
//        stopMotors();
//    }
//
//    public void moveLeft(double power) {
//        robot.getRightFront().setPower(power);
//        robot.getRightBack().setPower(-power);
//        robot.getLeftFront().setPower(-power);
//        robot.getLeftBack().setPower(power);
//    }
//
//    public void moveRight(double power, double dist) {
//        double initialPos = robot.getRightFront().getCurrentPosition();
//
//        while (robot.getRightFront().getCurrentPosition() > initialPos - dist) {
//            robot.getRightFront().setPower(-power);
//            robot.getRightBack().setPower(power);
//            robot.getLeftFront().setPower(power);
//            robot.getLeftBack().setPower(-power);
//        }
//        stopMotors();
//    }
//
//    public void moveRight(double power) {
//        robot.getRightFront().setPower(-power);
//        robot.getRightBack().setPower(power);
//        robot.getLeftFront().setPower(power);
//        robot.getLeftBack().setPower(-power);
//    }
//
//    public void moveForward(double power, double dist) {
//        double initialPos = robot.getRightFront().getCurrentPosition();
//
//        while (robot.getRightFront().getCurrentPosition() < initialPos + dist) {
//            robot.getRightFront().setPower(power);
//            robot.getRightBack().setPower(power);
//            robot.getLeftFront().setPower(power);
//            robot.getLeftBack().setPower(power);
//        }
//        stopMotors();
//    }
//
//    public void moveForward(double power) {
//        robot.getRightFront().setPower(power);
//        robot.getRightBack().setPower(power);
//        robot.getLeftFront().setPower(power);
//        robot.getLeftBack().setPower(power);
//    }
//
//    public void moveBackward(double power, double dist) {
//        double initialPos = robot.getRightFront().getCurrentPosition();
//
//        while (robot.getRightFront().getCurrentPosition() > initialPos - Math.abs(dist)) {
//            robot.getRightFront().setPower(-power);
//            robot.getRightBack().setPower(-power);
//            robot.getLeftFront().setPower(-power);
//            robot.getLeftBack().setPower(-power);
//        }
//        stopMotors();
//    }
//
//    public void moveBackward(double power) {
//        robot.getRightFront().setPower(-power);
//        robot.getRightBack().setPower(-power);
//        robot.getLeftFront().setPower(-power);
//        robot.getLeftBack().setPower(-power);
//    }
//
//    public void turnRight(double power, double angle) {
//        double initialAngle = getIntegratedAngle();
//
//        while (getIntegratedAngle() > initialAngle - angle + 5) {
//            robot.getRightFront().setPower(-power);
//            robot.getRightBack().setPower(-power);
//            robot.getLeftFront().setPower(power);
//            robot.getLeftBack().setPower(power);
//            telemetry.addData("angle", getIntegratedAngle());
//            telemetry.update();
//        }
//        stopMotors();
//
//    }
//
//    public void turnLeft(double power, double angle) {
//        double initialAngle = getIntegratedAngle();
//
//        while (getIntegratedAngle() < initialAngle + angle - 10) {
//            robot.getRightFront().setPower(power);
//            robot.getRightBack().setPower(power);
//            robot.getLeftFront().setPower(-power);
//            robot.getLeftBack().setPower(-power);
//            telemetry.addData("angle", getIntegratedAngle());
//            telemetry.update();
//        }
//        stopMotors();
//    }
//
//    /**
//     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
//     * It transforms the value from (-180, 180) to (-inf, inf).
//     * This code was taken and modified from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
//     *
//     * @return The integrated heading on the interval (-inf, inf).
//     */
//    private double getIntegratedAngle() {
//        double currentAngle = robot.imu.getAngularOrientation().firstAngle;
//        double deltaAngle = currentAngle - previousAngle;
//
//        if (deltaAngle < -180) {
//            deltaAngle += 360;
//        } else if (deltaAngle >= 180) {
//            deltaAngle -= 360;
//        }
//
//        integratedAngle += deltaAngle;
//        previousAngle = currentAngle;
//
//        return integratedAngle;
//    }

//    public void armSetPosition(double power, int ticks)
//    {
//        robot.getArm().setTargetPosition(ticks);
//        robot.getArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.getArm().setPower(power);
//        while (opModeIsActive() && robot.getTurret().isBusy())
//        {
//            telemetry.addData("current position", robot.getArm().getCurrentPosition());
//            telemetry.update();
//        }
//    }
}
