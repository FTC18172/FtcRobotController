package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.core.UpliftRobot;

@Autonomous(name = "StatesCycleBlue", group = "OpModes")
public class StatesCycleBlue extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {
        int location = robot.pipeline.location;
        robot.getTinyArm().setPosition(.1);
        if (location == 0 || location == -1) {
            robot.getWebcam().stopRecordingPipeline();

            robot.getTinyArm().setPosition(robot.armBottomLayer);

            moveBackward(0.5, 800);
            moveRight(0.5, 1000);

            robot.getBucketLatch().setPosition(robot.openBucketLatch);
            robot.safeSleep(1000);

            TurretPos(robot.turretAngleMid);

            robot.getTinyArm().setPosition(robot.armBasePos);

            moveLeft(0.6);
            robot.safeSleep(1600);

            moveForward(.5, 1550);

            robot.getBucketLatch().setPosition(.26);


        } else if (location == 1) {
            robot.getWebcam().stopRecordingPipeline();

            robot.getTinyArm().setPosition(robot.armMidLayer);

            moveBackward(0.5, 800);
            moveRight(0.5, 800);

            robot.getBucketLatch().setPosition(robot.openBucketLatch);
            robot.safeSleep(1000);

            TurretPos(robot.turretAngleMid);

            robot.getTinyArm().setPosition(robot.armBasePos);

            moveLeft(0.6);
            robot.safeSleep(1400);

            moveForward(.5, 1550);

            robot.getBucketLatch().setPosition(.26);

        } else if (location == 2) {
            robot.getWebcam().stopRecordingPipeline();

            robot.getTinyArm().setPosition(robot.armTopLayer);

            moveBackward(0.5, 800);
            moveRight(0.5, 1200);

            robot.getBucketLatch().setPosition(robot.openBucketLatch);
            robot.safeSleep(1000);

            TurretPos(robot.turretAngleMid);

            robot.getTinyArm().setPosition(robot.armBasePos);

            moveLeft(0.6);
            robot.safeSleep(1800);

            moveForward(.5, 1550);


            robot.getBucketLatch().setPosition(.26);

        }

        for (int i = 0; i < 2; i++) {

            while (robot.getBucketSensor().alpha() < 350) {
                robot.getTinyArm().setPosition(robot.armBasePos);
                moveForward(0.2);
                robot.getIntake().setPower(-0.7);
                if (robot.getBucketSensor().alpha() > 350) {
                    robot.getBucketLatch().setPosition(robot.closeBucketLatch);
                    stopMotors();
                    break;
                }
            }
            while (robot.getBottomSensor().alpha() < 450) {

                moveBackward(0.3);
                if (robot.getBottomSensor().alpha() > 450) {
                    stopMotors();
                    break;
                }

            }
            moveBackward(0.5, 1200);
            moveRight(0.5, 1200);

            robot.getIntake().setPower(-.1);
           // robot.safeSleep(1000);
            robot.getTinyArm().setPosition(robot.armTopLayer);
            robot.safeSleep(5000);

            TurretPos(robot.leftAngle);

            robot.getBucketLatch().setPosition(robot.openBucketLatch);
            robot.safeSleep(1000);

            TurretPos(robot.turretAngleMid);

            robot.getTinyArm().setPosition(robot.armBasePos);

            moveLeft(0.6);
            robot.safeSleep(1800);

            moveForward(.5, 1550);


            robot.getBucketLatch().setPosition(.26);

        }
    }




//        for( int i = 0; i < 3; i++)
//        {
//            while(robot.getBucketSensor().alpha() < 1000)
//            {
//                moveForward(0.5);
//                robot.getIntake().setPower(0.6);
//                if(robot.getBucketSensor().alpha() > 1000)
//                {
//                    stopMotors();
//                    break;
//                }
//            }
//            while(robot.getBottomSensor().alpha() < 1000)
//            {
//                robot.getIntake().setPower(-0.5);
//                moveBackward(0.5);
//                if(robot.getBucketSensor().alpha() > 1000)
//                {
//                    stopMotors();
//                    break;
//                }
//            }
//            moveBackward(0.5, 500);
//            setBucketUp();
//            blueTurretPos(blueTurretAngle);
//            armSetPosition(1, 1008);
//            setBucketHigh();
//            setBucketUp();
//            blueTurretPos(turretPosMid);
//            setBucketDown();
//        }
//        moveForward(1, 700);









    }


