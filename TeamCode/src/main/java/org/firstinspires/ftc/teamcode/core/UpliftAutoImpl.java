package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;

public class UpliftAutoImpl extends UpliftAuto {
    private double previousAngle = 0;
    private double integratedAngle = 0;

    public final double redTurretAngle = 2;
    public final double blueTurretAngle = 0.9;
    public final double turretPosMid = 1.43;



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

    public void stopMotors() {
        robot.getLeftFront().setPower(0);
        robot.getLeftBack().setPower(0);
        robot.getRightFront().setPower(0);
        robot.getRightBack().setPower(0);
    }

    public void topLayer() throws InterruptedException {
        robot.getBucket().setPosition(0.72);
        //bucket.setPosition(.65);
        robot.safeSleep(500);
        armSetPosition();
        robot.safeSleep(1000);
        robot.getBucket().setPosition(1);
        robot.safeSleep(500);
        armSetPosition();
        robot.safeSleep(500);
        robot.getBucket().setPosition(0.25);
    }

    public void middleLayer() throws InterruptedException {
        robot.getBucket().setPosition(0.75);
        robot.safeSleep(500);
        armSetPosition();
        robot.safeSleep(2000);
        robot.getBucket().setPosition(1);
        robot.safeSleep(1000);
        armSetPosition();
        robot.safeSleep(500);
        robot.getBucket().setPosition(0.25);
    }

    public void bottomLayer() throws InterruptedException {
        armSetPosition(1, 1008);
    }

    public void moveLeft(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() < initialPos + dist) {
            robot.getRightFront().setPower(power);
            robot.getRightBack().setPower(-power);
            robot.getLeftFront().setPower(-power);
            robot.getLeftBack().setPower(power);
        }
        stopMotors();
    }

    public void moveLeft(double power) {
        robot.getRightFront().setPower(power);
        robot.getRightBack().setPower(-power);
        robot.getLeftFront().setPower(-power);
        robot.getLeftBack().setPower(power);
    }

    public void moveRight(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() > initialPos - dist) {
            robot.getRightFront().setPower(-power);
            robot.getRightBack().setPower(power);
            robot.getLeftFront().setPower(power);
            robot.getLeftBack().setPower(-power);
        }
        stopMotors();
    }

    public void moveRight(double power) {
        robot.getRightFront().setPower(-power);
        robot.getRightBack().setPower(power);
        robot.getLeftFront().setPower(power);
        robot.getLeftBack().setPower(-power);
    }

    public void moveForward(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() < initialPos + dist) {
            robot.getRightFront().setPower(power);
            robot.getRightBack().setPower(power);
            robot.getLeftFront().setPower(power);
            robot.getLeftBack().setPower(power);
        }
        stopMotors();
    }

    public void moveForward(double power) {
        robot.getRightFront().setPower(power);
        robot.getRightBack().setPower(power);
        robot.getLeftFront().setPower(power);
        robot.getLeftBack().setPower(power);
    }

    public void moveBackward(double power, double dist) {
        double initialPos = robot.getRightFront().getCurrentPosition();

        while (robot.getRightFront().getCurrentPosition() > initialPos - Math.abs(dist)) {
            robot.getRightFront().setPower(-power);
            robot.getRightBack().setPower(-power);
            robot.getLeftFront().setPower(-power);
            robot.getLeftBack().setPower(-power);
        }
        stopMotors();
    }

    public void moveBackward(double power) {
        robot.getRightFront().setPower(-power);
        robot.getRightBack().setPower(-power);
        robot.getLeftFront().setPower(-power);
        robot.getLeftBack().setPower(-power);
    }

    public void turnRight(double power, double angle) {
        double initialAngle = getIntegratedAngle();

        while (getIntegratedAngle() > initialAngle - angle + 5) {
            robot.getRightFront().setPower(-power);
            robot.getRightBack().setPower(-power);
            robot.getLeftFront().setPower(power);
            robot.getLeftBack().setPower(power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();

    }

    public void turnLeft(double power, double angle) {
        double initialAngle = getIntegratedAngle();

        while (getIntegratedAngle() < initialAngle + angle - 10) {
            robot.getRightFront().setPower(power);
            robot.getRightBack().setPower(power);
            robot.getLeftFront().setPower(-power);
            robot.getLeftBack().setPower(-power);
            telemetry.addData("angle", getIntegratedAngle());
            telemetry.update();
        }
        stopMotors();
    }

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

    public void armSetPosition(double power, int ticks)
    {
        robot.getArm1().setTargetPosition(ticks);
        robot.getArm2().setTargetPosition(ticks);
        robot.getArm1().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getArm2().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getArm1().setPower(power);
        robot.getArm2().setPower(power);
        while (opModeIsActive() && robot.getArm1().isBusy() && robot.getArm2().isBusy())
        {
            telemetry.addData("arm1 current position", robot.getArm1().getCurrentPosition());
            telemetry.addData("arm2 current position", robot.getArm2().getCurrentPosition());
            telemetry.update();
        }
    }
    public void setBucketUp()
    {
        robot.getBucketLatch().setPosition(1);
        robot.getTinyArm().setPosition(0.5);
    }

    public void setBucketHigh()
    {
        robot.getBucketLatch().setPosition(1);
        robot.getTinyArm().setPosition(0.7);
    }

    public void setBucketMid()
    {
        robot.getBucketLatch().setPosition(1);
        robot.getTinyArm().setPosition(0.8);
    }

    public void setBucketLow()
    {
        robot.getBucketLatch().setPosition(1);
        robot.getTinyArm().setPosition(0.9);
    }
    public void blueTurretPos(double potentiometerPos)
    {

        if (robot.getPotentiometer().getVoltage() < potentiometerPos) {
            while (robot.getPotentiometer().getVoltage() < potentiometerPos) {
                robot.getTurret().setPower(0.1);
            }
            robot.getTurret().setPower(0);
            if (robot.getPotentiometer().getVoltage() > potentiometerPos) {
                while (robot.getPotentiometer().getVoltage() > potentiometerPos) {
                    robot.getTurret().setPower(-0.1);
                }
                robot.getTurret().setPower(0);
            }
        } else if (robot.getPotentiometer().getVoltage() > potentiometerPos) {
            while (robot.getPotentiometer().getVoltage() > potentiometerPos) {
                robot.getTurret().setPower(-0.1);
            }
            robot.getTurret().setPower(0);
        }
    }

}
