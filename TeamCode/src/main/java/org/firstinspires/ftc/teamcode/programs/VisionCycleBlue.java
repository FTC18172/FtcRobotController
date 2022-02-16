package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;

@Autonomous(name = "VisionCycleBlue", group = "OpModes")
public class VisionCycleBlue extends UpliftAutoImpl {

    UpliftRobot robot;

    @Override
    public void body() throws InterruptedException {
        int location = robot.pipeline.location;
        if (location == 0) {
            robot.getWebcam().stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5, 350);

            moveRight(0.5, 1100);

            turnLeft(0.5, 170);

            moveBackward(0.5, 75);
            stopMotors();

            bottomLayer();

        } else if (location == 1) {
            robot.getWebcam().stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5, 300);

            moveRight(0.5, 1200);

            turnLeft(0.5, 170);

            moveBackward(0.5, 100);
            stopMotors();

            middleLayer();

        } else if (location == 2) {
            robot.getWebcam().stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5, 300);

            moveRight(0.5, 1200);

            turnLeft(0.5, 175);

            moveBackward(0.5, 300);
            stopMotors();

            topLayer();

        }
        robot.getBucket().setPosition(.25);

        moveForward(.7,500);

        turnRight(.7,85);

        moveLeft(0.45);

        robot.safeSleep(1000);

        moveForward(.7,1300);

        moveForward(.2);

        robot.getIntake().setPower(.75);

        robot.safeSleep(3000);

        robot.getIntake().setPower(0);

        moveBackward(0.7, 2650);

        moveRight(0.7, 1000);

        turnLeft(0.7, 80);

        moveBackward(0.5, 100);

        topLayer();

        moveForward(.7,500);

        turnRight(.7,85);

        moveLeft(0.45);

        robot.safeSleep(1000);

        moveForward(.7,1300);

        moveForward(.2);

        robot.getIntake().setPower(.75);

        robot.safeSleep(3000);

        robot.getIntake().setPower(0);

        moveBackward(0.7, 2650);

        moveRight(0.7, 1000);

        turnLeft(0.7, 80);

        moveBackward(0.5, 100);

        topLayer();

        moveForward(.7,500);

        turnRight(.7,85);

        moveLeft(0.45);

        robot.safeSleep(1000);

        moveForward(.7,1300);

        moveForward(1, 500);





    }
}

