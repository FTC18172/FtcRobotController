package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;

@Autonomous(name = "VisionPhubBlue", group = "OpModes")
public class VisionPhubBlue extends UpliftAutoImpl {

    UpliftRobot robot;

    @Override
    public void body() throws InterruptedException {
        int location = robot.pipeline.location;
        if(location == 0){
            robot.getWebcam().stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5,350);

            moveRight(0.5,1100);

            turnLeft(0.5,180);

            moveBackward(0.5, 190);
            stopMotors();

            bottomLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(0.5,250);

            turnRight(.5,80);

            moveForward(.6,2800);
        }
        else if(location == 1){
            robot.getWebcam().stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5,300);

            moveRight(0.5,1200);

            turnLeft(0.5,175);

            moveBackward(0.5, 300);
            stopMotors();

            middleLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(0.5,220);

            turnRight(.5,78);

            moveForward(.6,2800);
        }
        else if(location == 2){
            robot.getWebcam().stopRecordingPipeline();

            moveRight(0.5, 100);

            moveForward(0.5,300);

            moveRight(0.5,1200);

            turnLeft(0.5,180);

            moveBackward(0.5, 400);
            stopMotors();

            topLayer();

            robot.getBucket().setPosition(0.5);

            moveForward(0.5,300);

            turnRight(.5,75);

            moveForward(.6,2800);
        }
        robot.getBucket().setPosition(.3);
    }
}