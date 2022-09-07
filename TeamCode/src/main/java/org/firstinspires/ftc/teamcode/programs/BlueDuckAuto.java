//package org.firstinspires.ftc.teamcode.programs;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
//
//@Autonomous(name = "BlueDuckAuto", group = "OpModes")
//public class BlueDuckAuto extends UpliftAutoImpl
//{
//    public void body() throws InterruptedException
//    {
//        int location = robot.pipeline.location;
//        if (location == 0 || location == -1) {
//            robot.getWebcam().stopRecordingPipeline();
//
//            robot.getTinyArm().setPosition(robot.armBottomLayer);
//
//            moveForward(0.6, 850);
//            moveRight(0.6, 950);
//
//            robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            robot.safeSleep(1000);
//
//            TurretPos(robot.turretAngleMid);
//
//            robot.getTinyArm().setPosition(robot.armBasePos);
//
//            turnLeft(0.5, 83);
//
//
//
//
//        } else if (location == 1) {
//            robot.getWebcam().stopRecordingPipeline();
//
//            robot.getTinyArm().setPosition(robot.armMidLayer);
//
//            moveForward(0.6, 850);
//            moveRight(0.6, 1300);
//
//            robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            robot.safeSleep(1000);
//
//            TurretPos(robot.turretAngleMid);
//
//            robot.getTinyArm().setPosition(robot.armBasePos);
//
//            turnLeft(0.5, 83);
//
//
//
//        } else if (location == 2) {
//            robot.getWebcam().stopRecordingPipeline();
//
//            robot.getTinyArm().setPosition(robot.armTopLayer);
//
//            moveForward(0.6, 850);
//            moveRight(0.6, 1470);
//
//            robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            robot.safeSleep(1000);
//
//            TurretPos(robot.turretAngleMid);
//
//            robot.getTinyArm().setPosition(robot.armBasePos);
//
//            turnLeft(0.5, 83);
//
//            moveForward(0.2, 100);
//
//
//
//        }
//
//
//
//        moveLeft(0.5, 2050);
//
//        moveLeft(0.6);
//        robot.safeSleep(700);
//
//        moveForward(0.2, 100);
//
//        moveForward(0.085);
//        robot.getIntake().setPower(0.3);
//        robot.safeSleep(12000);
//        stopMotors();
//    }
//
//}
