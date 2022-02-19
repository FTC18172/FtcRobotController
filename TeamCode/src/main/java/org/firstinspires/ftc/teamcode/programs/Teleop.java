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

    boolean aReleased = true;
    boolean bReleased = true;
    double capXPos = 0.45;
    double capYPos = 0.5;
    double capArmPos = 0.5;

    UpliftRobot robot;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {
        robot.getCapX().setPosition(0.45);
        robot.getCapY().setPosition(0.5);
        robot.getCapArm().setPosition(0.5);
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

        telemetry.addData("Freight", robot.getBucketSensor().alpha());
        telemetry.update();


        bReleased = setCapIn(bReleased);

        aReleased = setCapOut(aReleased);

//            DPAD_DOWN = setCapDown(DPAD_DOWN);
//
//            DPAD_UP = setCapUp(DPAD_UP);
//
//            DPAD_LEFT = setCapLeft(DPAD_LEFT);
//
//            DPAD_RIGHT = setCapRight(DPAD_RIGHT);


        moveCapRight();
//        moveCapUp();
//        moveCapDown();
        moveCapArmDown();
        moveCapArmUp();
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

    /*public void BucketRotateRight() throws InterruptedException {
        if (gamepad2.dpad_right)
        {
            motor.setPower(.5);
            motor.
        }
    }*/



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

//



    public void moveCapRight() {
        double currentPosition = capXPos + 0.003;
        if (gamepad1.dpad_right) {
            robot.getCapX().setPosition(currentPosition);
            capXPos = currentPosition;
        }
    }

//    public void moveCapUp() {
//        double currentPosition = capYPos - 0.01;
//        if (gamepad1.dpad_up) {
//            robot.getCapY().setPosition(currentPosition);
//            capYPos = currentPosition;
//        }



//    public void moveCapDown() {
//        double currentPosition = capYPos + 0.01;
//        if (gamepad1.dpad_down) {
//            robot.getCapY().setPosition(currentPosition);
//            capYPos = currentPosition;
//        }
//    }

    public void moveCapArmDown() {
        double currentPosition = capArmPos + 0.003;
        if (gamepad1.circle) {
            robot.getCapArm().setPosition(currentPosition);
            capArmPos = currentPosition;
        }
    }

    public void moveCapArmUp() {
        double currentPosition = capArmPos - 0.003;
        if (gamepad1.triangle) {
            robot.getCapArm().setPosition(currentPosition);
            capArmPos = currentPosition;
        }
    }


//

    public boolean setCapOut(boolean aReleased) {

        if (gamepad1.a || !aReleased) {
            robot.getCap().setPower(0.5);
            aReleased = false;

            if (!gamepad1.a) {
                aReleased = true;
                robot.getCap().setPower(0);
            }

        }
        return aReleased;

    }

    public boolean setCapIn(boolean bReleased) {
        if (gamepad1.b || !bReleased) {
            robot.getCap().setPower(-1);
            bReleased = false;

            if (!gamepad1.b) {
                bReleased = true;
                robot.getCap().setPower(0);
            }
        }
        return bReleased;
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

    public void stopMotors() {
        //arm.setPower(0);
    }

}
