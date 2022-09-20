package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

@TeleOp(name = "teleOp1", group = "Opmodes")
public class Teleop1 extends UpliftTele {

    UpliftRobot robot;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
    }

    @Override
    public void initAction() {
        robot.getTransfer().setPosition(0.65);
    }

    @Override
    public void bodyLoop() throws InterruptedException {

        double leftY = Range.clip(-gamepad2.left_stick_y, -1, 1);
        double rightX = Range.clip(gamepad2.right_stick_x, -1, 1);
        double leftX = Range.clip(gamepad2.left_stick_x, -1, 1);


//
        double angle = 90 - Math.toDegrees(UpliftMath.atan2UL(leftY, leftX));
        double magnitude = 0.8 * Range.clip(Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)), -1, 1);

        teleDrive(angle, magnitude, rightX, robot);

        shooterOn(1);

        shooterOff();

        intakeOn(-.5);


        moveTransfer();

        intakeOff();


    }

    @Override
    public void exit() {

    }

    public static void teleDrive ( double joystickAngle, double speedVal,
        double turnVal, UpliftRobot robot){
            double lfPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal + turnVal;
            double rfPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal - turnVal;
            double lbPow = sin(toRadians(joystickAngle) - (0.25 * PI)) * speedVal + turnVal;
            double rbPow = sin(toRadians(joystickAngle) + (0.25 * PI)) * speedVal - turnVal;
//
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

    public void shooterOn(double power)
    {
        if (gamepad2.right_trigger > 0)
        {
            robot.getShooter().setPower(power);
        }
    }

    public void shooterOff()
    {
        if(gamepad2.left_trigger > 0)
        {
            robot.getShooter().setPower(0);
        }
    }

    public void moveTransfer()
    {
        if(gamepad2.a)
        {
            robot.getTransfer().setPosition(0.125);
            robot.safeSleep(1000);
            robot.getTransfer().setPosition(0.65);

        }

    }

    public void intakeOn(double power)
    {
        if (gamepad2.y)
        {
            robot.getIntake().setPower(power);
        }
    }

    public void intakeOff()
    {
        if(gamepad2.b)
        {
            robot.getIntake().setPower(0);
        }
    }




}
