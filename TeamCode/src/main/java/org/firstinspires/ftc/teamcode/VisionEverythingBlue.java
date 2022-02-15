package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.UpliftAuto;
import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "VisionEverythingBlue", group = "OpModes")
public class VisionEverythingBlue extends UpliftAutoImpl {
   
    @Override
    public void body() throws InterruptedException {
        int location = robot.pipeline.location;
        if(location == 0 || location == -1)
        {
            robot.getWebcam().stopRecordingPipeline();

            robot.safeSleep(7000);

            moveForward(0.5, 420);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 175);

            moveBackward(0.5, 180);
            stopMotors();

            bottomLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(0.5, 50);
        }
        else if(location == 1)
        {
            robot.getWebcam().stopRecordingPipeline();

            robot.safeSleep(7000);

            moveForward(0.5, 420);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 175);

            moveBackward(0.5, 300);
            stopMotors();

            middleLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(0.5, 100);
        }
        else if(location == 2)
        {
            robot.getWebcam().stopRecordingPipeline();

            robot.safeSleep(7000);

            moveForward(0.5, 500);

            moveLeft(0.5, 1000);

            turnLeft(0.5, 175);

            moveBackward(0.5, 420);
            stopMotors();

            topLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(0.5, 300);
        }

        robot.getBucket().setPosition(.5);
        moveLeft(0.5);
        robot.safeSleep(3000);

        moveForward(0.2, 100);

        moveForward(0.1);
        robot.getDuck().setPower(0.3);
        robot.safeSleep(5000);
        stopMotors();

        robot.getDuck().setPower(.65);
        robot.safeSleep(1000);
        robot.getDuck().setPower(0);

        moveBackward(0.5, 300);

        turnRight(0.5, 80);

        moveBackward(0.5);
        Thread.sleep(300);

        moveBackward(0.2);
        Thread.sleep(800);

        moveForward(0.5, 1400);

        turnRight(0.5, 5);

        moveForward(0.5, 3500);

        robot.getBucket().setPosition(.3);

    }
}

