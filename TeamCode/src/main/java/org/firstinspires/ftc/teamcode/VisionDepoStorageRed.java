package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "VisionDepoStorageRed", group = "OpModes")
public class VisionDepoStorageRed extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {

        int location = robot.pipeline.location;
        if(location == 0 || location == -1 )
        {
            robot.safeSleep(7000);

            moveRight(0.5, 100);

            moveForward(0.5, 300);

            moveRight(0.5, 1250);

            turnLeft(0.5, 180);

            moveBackward(0.4, 260);
            stopMotors();

            bottomLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(.3, 120);

            moveRight(.5, 1600);

            turnRight(.3, 50);

            moveForward(.3, 350);

            moveForward(0.13);
            robot.getDuck().setPower(-0.3);
            Thread.sleep(4000);
            stopMotors();

            robot.getDuck().setPower(-.65);
            Thread.sleep(1000);
            robot.getDuck().setPower(0);

            moveBackward(0.5, 300);

            turnLeft(0.5, 140);

            moveBackward(0.5);
            Thread.sleep(600);

            moveBackward(0.2);
            Thread.sleep(800);

            moveLeft(0.5, 700);

            robot.getBucket().setPosition(.25);

        }
        else if(location == 1)
        {
            robot.safeSleep(7000);

            moveRight(0.5, 100);

            moveForward(0.5, 300);

            moveRight(0.5, 1250);

            turnLeft(0.5, 180);

            moveBackward(0.4, 350);
            stopMotors();

            middleLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(.3, 150);

            moveRight(.5, 1600);

            turnRight(.3, 45);

            moveForward(.3, 350);

            moveForward(0.13);
            robot.getDuck().setPower(-0.3);
            Thread.sleep(4000);
            stopMotors();

            robot.getDuck().setPower(-.65);
            Thread.sleep(1000);
            robot.getDuck().setPower(0);

            moveBackward(0.5, 300);

            turnLeft(0.5, 140);

            moveBackward(0.5);
            Thread.sleep(600);

            moveBackward(0.2);
            Thread.sleep(800);

            moveLeft(0.5, 700);

            robot.getBucket().setPosition(.25);

        }
        else if(location == 2)
        {
            robot.safeSleep(7000);

            robot.getWebcam().stopRecordingPipeline();
            moveRight(0.5, 100);

            moveForward(0.5, 400);

            moveRight(0.5, 1300);

            turnLeft(0.5, 180);

            moveBackward(0.5, 350);
            stopMotors();

            topLayer();

            robot.getBucket().setPosition(0.5);

            robot.getBucket().setPosition(.4);

            moveForward(0.5, 200);

            moveRight(.5, 1700);

            turnRight(.3, 45);

            moveForward(.2, 350);

            moveForward(0.13);
            robot.getDuck().setPower(-0.3);
            Thread.sleep(4000);
            stopMotors();

            robot.getDuck().setPower(-.65);
            Thread.sleep(1000);
            robot.getDuck().setPower(0);

            moveBackward(0.5, 300);

            turnLeft(0.5, 140);

            moveBackward(0.5);
            Thread.sleep(600);

            moveBackward(0.2);
            Thread.sleep(800);

            moveLeft(0.5, 700);

            robot.getBucket().setPosition(.3);

        }
    }
}
