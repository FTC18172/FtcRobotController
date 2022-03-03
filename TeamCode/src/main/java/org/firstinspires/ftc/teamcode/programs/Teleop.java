package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

@TeleOp(name = "teleOp", group = "Opmodes")
public class Teleop extends UpliftTele {


    UpliftRobot robot;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction()
    {

    }

    @Override
    public void bodyLoop() throws InterruptedException {
        robot.getWebcam().stopRecordingPipeline();
        robot.getWebcam().closeCameraDevice();
        double leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        double rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
        double leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

        if (gamepad1.right_bumper) {
            leftY /= 2;
            rightX /= 2;
            leftX /= 2;
        }


        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.8 * Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, robot);

        intakeOn();

        checkFreight(robot.getBucketSensor());

        moveDuck();

        armDown();
        sharedArmDown();
        topLayer();
        sharedHub();
        ting();

        robot.getPotentiometer();

        telemetry.addData("Freight", robot.getBucketSensor().alpha());

        telemetry.addData("Potentiometer", robot.getPotentiometer().getVoltage());
        telemetry.update();

    }

    @Override
    public void exit() {

    }

    public static void teleDrive(double joystickAngle, double speedVal, double turnVal, UpliftRobot robot) {
        double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
        double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
        double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
        double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;

        // find max total input out of the 4 motors
        double maxVal = abs(lfPow);
        if (abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if (abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if (abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if (maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        robot.getLeftFront().setPower(lfPow / maxVal);
        robot.getLeftBack().setPower(lbPow / maxVal);
        robot.getRightBack().setPower(rbPow / maxVal);
        robot.getRightFront().setPower(rfPow / maxVal);
    }
    public void armDown() throws InterruptedException {
        if (gamepad2.y)
        {
            if( gamepad1.atRest())
            {
                robot.getBucket().setPosition(1);
                robot.safeSleep(500);
                robot.getArm().setPosition(.03);
                robot.safeSleep(500);
                robot.getBucket().setPosition(0.3);
            }
        }
    }

    public void sharedArmDown() throws InterruptedException {
        if (gamepad2.x) {
            robot.getBucket().setPosition(1);
            Thread.sleep(500);
            robot.getArm().setPosition(.024);
            Thread.sleep(500);
            robot.getBucket().setPosition(0.3);
        }
    }

    public void ting() throws InterruptedException {
        if (gamepad2.dpad_down) {

            robot.getArm().setPosition(.024);
            Thread.sleep(500);
            robot.getBucket().setPosition(0.3);
        }
    }



    public void sharedHub() throws InterruptedException {
        if (gamepad2.a) {
            robot.getBucket().setPosition(0.8);
            robot.getArm().setPosition(0.95);

        }
    }

    public void topLayer() throws InterruptedException {
        if (gamepad2.b) {
            robot.getBucket().setPosition(0.62);
            robot.getArm().setPosition(0.6);
        }

    }

    public void checkFreight(ColorSensor sensor) throws InterruptedException {
        if (sensor.alpha() > 1000) {
            gamepad1.rumble(1000);
        }
    }


    public void intakeOn() {
        robot.getIntake().setPower(.77 * Range.clip(gamepad2.left_stick_y, -1, 1));
    }


    public void moveDuck() {
        robot.getDuck().setPower(.7 * Range.clip(gamepad2.left_stick_x, -1, 1));

    }


}
