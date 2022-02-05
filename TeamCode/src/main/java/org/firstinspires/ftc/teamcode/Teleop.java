package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.UpliftMath;
import org.openftc.easyopencv.OpenCvCamera;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

@TeleOp(name = "teleOp", group = "Opmodes")
public class Teleop extends UpliftTele {
    UpliftRobot robot;
    DcMotor lf, rf, lb, rb;
    DcMotor intake, duck;
    Servo bucket, arm;
    CRServo capX, capY, cap;
    ColorSensor bucketSensor;
    OpenCvCamera webcam;
    boolean aReleased = false;
    boolean bReleased = false;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        lb = robot.leftBack;
        rf = robot.rightFront;
        rb = robot.rightBack;
        intake = robot.intake;
        duck = robot.duck;
        arm = robot.arm;
        bucket = robot.bucket;
        bucketSensor = robot.bucketSensor;
        webcam = robot.webcam;
        capX = robot.capX;
        capY = robot.capY;
        cap = robot.capOut;
    }

    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() throws InterruptedException {
        webcam.stopRecordingPipeline();
        webcam.closeCameraDevice();
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

        moveDuck();

        armDown();
        sharedArmDown();
        topLayer();
        sharedHub();
        ting();

        telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("integrated Angle", getIntegratedAngle());
        telemetry.addData("Freight", bucketSensor.alpha());
        telemetry.update();

//        if(gamepad1.a || !aReleased)
            aReleased = setCapIn(aReleased);
//        else if (gamepad1.b || !bReleased)
            bReleased = setCapOut(bReleased);
        setCapX();
        setCapY();

    }

    @Override
    public void exit() {

    }

    public static void
    teleDrive(double joystickAngle, double speedVal, double turnVal, UpliftRobot robot) {
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
        robot.leftFront.setPower(lfPow / maxVal);
        robot.leftBack.setPower(lbPow / maxVal);
        robot.rightBack.setPower(rbPow / maxVal);
        robot.rightFront.setPower(rfPow / maxVal);
    }

    public void armDown() throws InterruptedException {
        if (gamepad2.y) {
            bucket.setPosition(1);
            Thread.sleep(500);
            arm.setPosition(.03);
            Thread.sleep(500);
            bucket.setPosition(0.30);
        }
    }

    public void sharedArmDown() throws InterruptedException {
        if (gamepad2.x) {
            bucket.setPosition(1);
            Thread.sleep(500);
            arm.setPosition(.024);
            Thread.sleep(500);
            bucket.setPosition(0.35);
        }
    }

    public void ting() throws InterruptedException {
        if (gamepad2.dpad_down) {

            arm.setPosition(.024);
            Thread.sleep(500);
            bucket.setPosition(0.22);
        }
    }

    public void sharedHub() throws InterruptedException {
        if (gamepad2.a) {
            bucket.setPosition(0.8);
            arm.setPosition(0.95);

        }
    }

    public void topLayer() throws InterruptedException {
        if (gamepad2.b) {
            bucket.setPosition(0.62);
            arm.setPosition(0.6);
        }

    }

    public void setCapX()
    {
        if(gamepad1.dpad_right)
        {
            capX.setPower(0.25);
        }
        else if(gamepad1.dpad_left)
        {
            capX.setPower(-0.25);
        }
    }

    public void setCapY()
    {
        if(gamepad1.dpad_up)
        {
            capY.setPower(0.25);
        }
        else if(gamepad1.dpad_down)
        {
            capY.setPower(-0.25);
        }
    }

    public boolean setCapOut(boolean aReleased)
    {

        if(gamepad1.a || !aReleased)
        {
            cap.setPower(0.5);
            aReleased = false;

            if(!gamepad1.a )
            {
                aReleased = true;
                cap.setPower(0);
            }

        }
        return aReleased;

    }

    public boolean setCapIn(boolean bReleased)
    {
        if (gamepad1.b || !bReleased)
        {
            cap.setPower(-0.5);
            bReleased = false;

            if(!gamepad1.a )
            {
                bReleased = true;
                cap.setPower(0);
            }
        }
        return bReleased;
    }


    public void intakeOn() {
        intake.setPower(.7 * Range.clip(gamepad2.left_stick_y, -1, 1));
    }


    public void moveDuck() {
        duck.setPower(.7 * Range.clip(gamepad2.left_stick_x, -1, 1));

    }

    public void stopMotors() {
        //arm.setPower(0);
    }

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
