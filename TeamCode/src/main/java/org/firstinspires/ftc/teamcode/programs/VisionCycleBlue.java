package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.core.UpliftRobot;

@Autonomous(name = "VisionCycleBlue", group = "OpModes")
public class VisionCycleBlue extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {
        int location = robot.pipeline.location;
        if (location == 0) {
            robot.getWebcam().stopRecordingPipeline();

            setBucketUp();
            blueTurretPos(blueTurretAngle);
            armSetPosition(1, 1008);
            setBucketLow();



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






    }
}

