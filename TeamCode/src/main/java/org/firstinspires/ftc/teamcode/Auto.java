package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.toolkit.UpliftMath;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

@Autonomous(name = "Auto", group = "OpModes")
public class Auto extends UpliftAuto {
    UpliftRobot robot;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;


    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);
        lf = robot.leftFront;
        rf = robot.rightFront;
        lb = robot.leftBack;
        rb = robot.rightBack;

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {

        driveToPosition(0, 2, 0.5, 1);
    }

    @Override
    public void exit() throws InterruptedException {

    }
    public void driveTowards(double speedVal, double relativeAngleToPoint) {
        double lfPow = sin(toRadians(90 - relativeAngleToPoint) + (0.25 * PI)) * speedVal;
        double rfPow = sin(toRadians(90 - relativeAngleToPoint) - (0.25 * PI)) * speedVal;
        double lbPow = sin(toRadians(90 - relativeAngleToPoint) - (0.25 * PI)) * speedVal;
        double rbPow = sin(toRadians(90 - relativeAngleToPoint) + (0.25 * PI)) * speedVal;

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
        lf.setPower(lfPow / maxVal);
        rf.setPower(rfPow / maxVal);
        lb.setPower(lbPow / maxVal);
        rb.setPower(rbPow / maxVal);
    }

    public void stopMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    public void driveToPosition(double x, double y, double speedVal, double tolerance) {
        double xDist = y - robot.worldX;
        double yDist = x - robot.worldY;
        double distance = hypot(xDist,yDist);
        double angle = Math.toDegrees(UpliftMath.atan2UL(yDist,xDist));
        telemetry.addData("y position", yDist);
        telemetry.update();
        Log.i("y position", yDist + "");
        Log.i("angle", angle + "");
        while(distance > tolerance && opModeIsActive()) {
            driveTowards(speedVal, angle);
            xDist = y - robot.worldX;
            yDist = x - robot.worldY;
            distance = hypot(xDist,yDist);
            angle = Math.toDegrees(UpliftMath.atan2UL(yDist,xDist));
            telemetry.addData("y position", yDist);
            telemetry.update();
            Log.i("y position", yDist + "");
            Log.i("angle", angle + "");
        }
        stopMotors();
    }
}