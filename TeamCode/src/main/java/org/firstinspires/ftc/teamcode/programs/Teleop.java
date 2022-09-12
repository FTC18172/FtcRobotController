package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

//@TeleOp(name = "teleOp", group = "Opmodes")
//public class Teleop extends UpliftTele {


//    UpliftRobot robot;

    //
//    double capYPos = 0.5;
//
//
//
//    @Override
//    public void initHardware() {
//        robot = new UpliftRobot(this);
//    }
//
//    @Override
//    public void initAction() {
//
//    }

    //
//    @Override
//    public void initAction()
//    {
//        //robot.getTinyArm().setPosition(robot.armBasePos);
//    }
//
//    @Override
//    public void bodyLoop() throws InterruptedException {
////        robot.getWebcam().stopRecordingPipeline();
////        robot.getWebcam().closeCameraDevice();
//        double leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
//        double rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
//        double leftX = Range.clip(gamepad1.left_stick_x, -1, 1);
//
//
////
//        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
//        double magnitude = 0.8 * Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);
//
//        teleDrive(angle, magnitude, rightX, robot);}
//
//        intakeOn();
//
//        checkFreight(robot.getBucketSensor());
//
//
//        sharedHub();
//
//        armReset();
//        topLayer();
//
//        //middleTest();
//
//
//        moveCapUp();
//        moveCapDown();
//
//        turretLeft();
//        turretRight();
//        turretMiddle();
//
//        latchUp();
//        latchDown();
//
//        slowIntake1();
//        slowIntake2();
//
//
////        telemetry.addData("cap", robot.getCap());
//        telemetry.addData("Potentiometer", robot.getPotentiometer().getVoltage());
////        telemetry.addData("BottomSensor", robot.getBottomSensor().alpha());
//
//        telemetry.addData("right front motor", robot.getRightFront().getCurrentPosition());
//        telemetry.addData("left front motor", robot.getLeftFront().getCurrentPosition());
//        telemetry.addData("right back motor", robot.getRightBack().getCurrentPosition());
//        telemetry.addData("left back motor", robot.getLeftBack().getCurrentPosition());
//
//        telemetry.update();
//
//
//
//
//    }
//
//        @Override
//        public void exit () {
//        }
//
//        public static void teleDrive ( double joystickAngle, double speedVal,
//        double turnVal, UpliftRobot robot){
//            double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
//            double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
//            double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
//            double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;
////
//            // find max total input out of the 4 motors
//            double maxVal = abs(lfPow);
//            if (abs(rfPow) > maxVal) {
//                maxVal = abs(rfPow);
//            }
//            if (abs(lbPow) > maxVal) {
//                maxVal = abs(lbPow);
//            }
//            if (abs(rbPow) > maxVal) {
//                maxVal = abs(rbPow);
//            }
//
//            if (maxVal < (1 / sqrt(2))) {
//                maxVal = 1 / sqrt(2);
//            }
//
//            // set the scaled powers
//            robot.getLeftFront().setPower(lfPow / maxVal);
//            robot.getLeftBack().setPower(lbPow / maxVal);
//            robot.getRightBack().setPower(rbPow / maxVal);
//            robot.getRightFront().setPower(rfPow / maxVal);
//        }
//
//    public void topLayer() throws InterruptedException
//    {
//
//          if (gamepad2.a)
//          {
//
//          }
//        // move arm up to park auto
//        if (gamepad2.b)

//        {
//            robot.getTinyArm().setPosition(robot.armTopLayer);
//
//        }
//
//    }
//
//    public void sharedHub() throws InterruptedException
//    {
//        // move arm to shared position
//        if (gamepad2.a)
//        {
//            robot.getTinyArm().setPosition(robot.armBottomLayer);
//
//        }
//    }
//
//    public void armReset() throws InterruptedException
//    {
//        // resets positions
//        if (gamepad2.y)
//        {
//            if( gamepad1.atRest())
//            {
//
//                robot.getBucketLatch().setPosition(robot.openBucketLatch);
//                robot.safeSleep(1000);
//                robot.getTinyArm().setPosition(robot.armBasePos);
//                robot.safeSleep(1000);
//                robot.getBucketLatch().setPosition(robot.openBucketLatch);
//
//
//            }
//        }
//    }
//
//
//    public void latchUp(){
//        if(gamepad2.dpad_up)
//        {
//            if(gamepad1.atRest())
//            {
//                robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            }
//        }
//    }
//    public void latchDown()
//    {
//        if(gamepad2.dpad_down)
//        {
//            if(gamepad1.atRest())
//            {
//                robot.getBucketLatch().setPosition(robot.closeBucketLatch);
//            }
//        }
//    }
//
//    public void slowIntake1()
//    {
//        if(gamepad2.dpad_left)
//        {
//            robot.getIntake().setPower(0.7);
//        }
//    }
//
//    public void slowIntake2()
//    {
//        if(gamepad2.dpad_right)
//        {
//            robot.getIntake().setPower(-0.7);
//        }
//    }
//
//    public void turretMiddle(){
//        if(gamepad2.x)
//        {
//            if (gamepad1.atRest())
//            {
//                robot.getTinyArm().setPosition(robot.armTurretPos);
//                if (robot.getPotentiometer().getVoltage() < robot.turretAngleMid)
//                {
//                    while (robot.getPotentiometer().getVoltage() < robot.turretAngleMid)
//                    {
//                        robot.getTurret().setPower(0.17);
//                    }
//                    robot.getTurret().setPower(0);
//                    if (robot.getPotentiometer().getVoltage() > robot.turretAngleMid)
//                    {
//                        while (robot.getPotentiometer().getVoltage() > robot.turretAngleMid)
//                        {
//                            robot.getTurret().setPower(-0.17);
//                        }
//                        robot.getTurret().setPower(0);
//                    }
//                } else if (robot.getPotentiometer().getVoltage() > robot.turretAngleMid)
//                {
//                    while (robot.getPotentiometer().getVoltage() > robot.turretAngleMid)
//                    {
//                        robot.getTurret().setPower(-0.17);
//                    }
//                    robot.getTurret().setPower(0);
//                }
//                //robot.getTinyArm().setPosition(robot.armBasePos);
//
//            }
//
////            robot.getBucketLatch().setPosition(robot.closeBucketLatch);
//        }
//    }
//
//    public void turretRight()
//    {
//        if(gamepad2.right_bumper)
//        {
//            if(gamepad1.atRest())
//            {
//                robot.getTinyArm().setPosition(robot.armTurretPos);
//                if (robot.getPotentiometer().getVoltage() < 1.78)
//                {
//                    while (robot.getPotentiometer().getVoltage() < 1.78)
//                    {
//                        robot.getTurret().setPower(0.45);
//                    }
//                    robot.getTurret().setPower(0);
//                    if (robot.getPotentiometer().getVoltage() > 1.78)
//                    {
//                        while (robot.getPotentiometer().getVoltage() > 1.78)
//                        {
//                            robot.getTurret().setPower(-0.45);
//                        }
//                        robot.getTurret().setPower(0);
//                    }
//                } else if (robot.getPotentiometer().getVoltage() > 1.78)
//                {
//                    while (robot.getPotentiometer().getVoltage() > 1.78)
//                    {
//                        robot.getTurret().setPower(-0.45);
//                    }
//                    robot.getTurret().setPower(0);
//                }
//                //robot.getTinyArm().setPosition(robot.armBasePos);
//            }
//        }
//    }
//
//    public void turretLeft()
//    {
//        if(gamepad2.left_bumper)
//        {
//            if(gamepad1.atRest())
//            {
//                robot.getTinyArm().setPosition(robot.armTurretPos);
//                if (robot.getPotentiometer().getVoltage() < 1)
//                {
//                    while (robot.getPotentiometer().getVoltage() < 1)
//                    {
//                        robot.getTurret().setPower(0.45);
//                    }
//                    robot.getTurret().setPower(0);
//                    if (robot.getPotentiometer().getVoltage() > 1)
//                    {
//                        while (robot.getPotentiometer().getVoltage() > 1)
//                        {
//                            robot.getTurret().setPower(-0.45);
//                        }
//                        robot.getTurret().setPower(0);
//                    }
//                } else if (robot.getPotentiometer().getVoltage() > 1)
//                {
//                    while (robot.getPotentiometer().getVoltage() > 1)
//                    {
//                        robot.getTurret().setPower(-0.45);
//                    }
//                    robot.getTurret().setPower(0);
//                }
//                //robot.getTinyArm().setPosition(robot.armBasePos);
//
//            }
//        }
//    }
//
//
//    public void checkFreight (ColorSensor sensor) throws InterruptedException {
//
//        if (sensor.alpha() > 350) {
//            robot.getBucketLatch().setPosition(robot.closeBucketLatch);
//            gamepad1.rumble(1000);
//        }
//    }
//
//
//    public void intakeOn() {
//        robot.getIntake().setPower(-.77 * Range.clip(gamepad2.left_stick_y, -1, 1));
//    }
//
//    public void moveCapUp() {
//        double currentPosition = capYPos + 0.002;
//        if (gamepad1.dpad_up)
//        {
//            robot.getCap().setPosition(currentPosition);
//            capYPos = currentPosition;
//        }
//    }
//
//    public void moveCapDown() {
//        double currentPosition = capYPos - 0.002;
//        if (gamepad1.left_bumper) {
//            robot.getCap().setPosition(currentPosition);
//            capYPos = currentPosition;
//        }
//    }
//
//}
//
//
//
//
//
//

