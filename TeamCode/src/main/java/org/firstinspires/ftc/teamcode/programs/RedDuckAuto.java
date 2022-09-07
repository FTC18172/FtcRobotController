//package org.firstinspires.ftc.teamcode.programs;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
//
//@Autonomous(name = "RedDuckAuto", group = "OpModes")
//public class RedDuckAuto extends UpliftAutoImpl {
//    public void body() throws InterruptedException {
//        int location = robot.pipeline.location;
//        if (location == 0 || location == -1) {
//            robot.getWebcam().stopRecordingPipeline();
//
//            robot.getTinyArm().setPosition(robot.armBottomLayer);
//
//            moveForward(0.6, 850);
//            moveLeft(0.6, 900);
//
//            robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            robot.safeSleep(1000);
//
//            TurretPos(robot.turretAngleMid);
//
//            robot.getTinyArm().setPosition(robot.armBasePos);
//
//
//        } else if (location == 1) {
//            robot.getWebcam().stopRecordingPipeline();
//
//            robot.getTinyArm().setPosition(robot.armMidLayer);
//
//            moveForward(0.6, 850);
//            moveLeft(0.6, 930);
//
//            robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            robot.safeSleep(1000);
//
//            TurretPos(robot.turretAngleMid);
//
//            robot.getTinyArm().setPosition(robot.armBasePos);
//
//
//        } else if (location == 2) {
//            robot.getWebcam().stopRecordingPipeline();
//
//            robot.getTinyArm().setPosition(robot.armTopLayer);
//
//            moveForward(0.6, 850);
//            moveLeft(0.6, 1070);
//
//            robot.getBucketLatch().setPosition(robot.openBucketLatch);
//            robot.safeSleep(1000);
//
//            TurretPos(robot.turretAngleMid);
//
//            robot.getTinyArm().setPosition(robot.armBasePos);
//
//
//        }
//        moveRight(0.5, 500);
//        moveBackward(0.5, 1700);
//
//        turnRight(0.5, 140);
//
//        moveForward(0.083);
//
//        robot.getIntake().setPower(-0.4);
//        Thread.sleep(7500);
//        stopMotors();
//
//        robot.getIntake().setPower(-.65);
//        Thread.sleep(1000);
//        robot.getIntake().setPower(0);
//
//
//        moveBackward(0.5, 300);
//
//        turnLeft(0.5, 140);
//
//        moveBackward(0.5);
//        Thread.sleep(600);
//
//        moveBackward(0.2);
//        Thread.sleep(800);
//
//        moveLeft(0.5, 700);
//    }
//}