package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    DcMotor lf, rf, lb, rb;
    DcMotor intake, duck, arm;
    Servo bucket;
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
    }
     
    @Override
    public void initAction() {

    }

    @Override
    public void bodyLoop() throws InterruptedException {
        double leftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        double rightX = Range.clip(gamepad1.right_stick_x, -1, 1);
        double leftX = Range.clip(gamepad1.left_stick_x, -1, 1);

        if (gamepad1.a){
            leftY /= 2;
            rightX /= 2;
            leftX /= 2;
        }


        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.6 * Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, robot);

        intakeOn();

        moveDuck();

        bucketDown();
        bucketDrop();

        //armUp();
        //armDown();

        telemetry.addData("angle", robot.imu.getAngularOrientation().firstAngle);
        telemetry.update();



        //armPositionUp();
        //armPositionDown();

        arm.setPower(-Range.clip(gamepad2.right_stick_y, -0.2, 0.2));


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
        if(abs(rfPow) > maxVal) {
            maxVal = abs(rfPow);
        }
        if(abs(lbPow) > maxVal) {
            maxVal = abs(lbPow);
        }
        if(abs(rbPow) > maxVal) {
            maxVal = abs(rbPow);
        }

        if(maxVal < (1 / sqrt(2))) {
            maxVal = 1 / sqrt(2);
        }

        // set the scaled powers
        robot.leftFront.setPower(lfPow / maxVal);
        robot.leftBack.setPower(lbPow / maxVal);
        robot.rightBack.setPower(rbPow / maxVal);
        robot.rightFront.setPower(rfPow / maxVal);
    }

    public void bucketDown()
    {
        if(gamepad2.x)
        {
            bucket.setPosition(0.33);
        }
    }

    public void bucketDrop()
    {
        if(gamepad2.y)
        {
            bucket.setPosition(1);
        }
    }

    public void armUp() throws InterruptedException
    {
        if(gamepad2.a)
        {
            arm.setPower(0.1);
            Thread.sleep(1900);
            stopMotors();
        }
    }

    public void armDown() throws InterruptedException
    {
        if(gamepad2.b)
        {
            arm.setPower(-0.1);
            Thread.sleep(1800);
            stopMotors();
        }

    }


    public void intakeOn() {
        intake.setPower(.6 * Range.clip(gamepad2.left_stick_y, -1, 1));
    }


    public void moveDuck() {
        duck.setPower(.7 * Range.clip(gamepad2.left_stick_x, -1, 1));

    }
    public void stopMotors()
    {
        arm.setPower(0);
    }
}
