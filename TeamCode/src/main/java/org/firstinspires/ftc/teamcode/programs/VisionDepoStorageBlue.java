//package org.firstinspires.ftc.teamcode.programs;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
//
//@Autonomous(name = "VisionDepoStorageBlue", group = "OpModes")
//public class VisionDepoStorageBlue extends UpliftAutoImpl {
//
//    @Override
//    public void body() throws InterruptedException {
//       int location = robot.pipeline.location;
//        if(location == 0 || location == -1 )
//        {
//            robot.getWebcam().stopRecordingPipeline();
//
//            moveForward(0.5, 420);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 175);
//
//            moveBackward(0.5, 180);
//            stopMotors();
//
//            bottomLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(0.5, 50);
//        }
//        else if(location == 1)
//        {
//            robot.getWebcam().stopRecordingPipeline();
//
//            moveForward(0.5, 420);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 175);
//
//            moveBackward(0.5, 300);
//            stopMotors();
//
//            middleLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(0.5, 100);
//        }
//        else if(location == 2)
//        {
//            robot.getWebcam().stopRecordingPipeline();
//
//            moveForward(0.5, 500);
//
//            moveLeft(0.5, 1000);
//
//            turnLeft(0.5, 175);
//
//            moveBackward(0.5, 420);
//            stopMotors();
//
//            topLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(0.5, 300);
//        }
//
//        moveLeft(0.5);
//        robot.safeSleep(3000);
//
//        moveForward(0.2, 100);
//
//        moveForward(0.15);
//        robot.getDuck().setPower(0.3);
//        robot.safeSleep(6000);
//        stopMotors();
//
//        robot.getDuck().setPower(.65);
//        robot.safeSleep(1000);
//        robot.getDuck().setPower(0);
//
//        moveBackward(0.5, 800);
//
//        robot.getBucket().setPosition(.3);
//
//    }
//}
