////package org.firstinspires.ftc.teamcode.programs;
////
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
//
//@Autonomous(name = "VisionEverythingRed", group = "OpModes")
//public class VisionEverythingRed extends UpliftAutoImpl {
//
//    @Override
//    public void body() throws InterruptedException {
//        int location = robot.pipeline.location;
//        if(location == 0 || location == -1)
//        {
//            moveRight(0.5, 100);
//
//            moveForward(0.5, 300);
//
//            moveRight(0.5, 1250);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.4, 260);
//            stopMotors();
//
//            bottomLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(.3, 120);
//
//            moveRight(.5, 1600);
//
//            turnRight(.3, 50);
//
//            moveForward(.3, 350);
//
//            moveForward(0.13);
//            robot.getDuck().setPower(-0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            robot.getDuck().setPower(-.65);
//            Thread.sleep(1000);
//            robot.getDuck().setPower(0);
//
//            moveBackward(0.5, 300);
//
//            turnLeft(0.5, 140);
//
//            moveBackward(0.5);
//            Thread.sleep(600);
//
//            moveBackward(0.2);
//            Thread.sleep(800);
//
//            moveForward(0.5, 1250);
//
//            turnLeft(0.5, 12);
//
//            moveForward(0.5, 4000);
//
//        }
//        else if(location == 1)
//        {
//            moveRight(0.5, 100);
//
//            moveForward(0.5, 300);
//
//            moveRight(0.5, 1250);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.4, 350);
//            stopMotors();
//
//            middleLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(.3, 150);
//
//            moveRight(.5, 1600);
//
//            turnRight(.3, 45);
//
//            moveForward(.3, 350);
//
//            moveForward(0.13);
//            robot.getDuck().setPower(-0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            robot.getDuck().setPower(-.65);
//            Thread.sleep(1000);
//            robot.getDuck().setPower(0);
//
//            moveBackward(0.5, 300);
//
//            turnLeft(0.5, 140);
//
//            moveBackward(0.5);
//            Thread.sleep(600);
//
//            moveBackward(0.2);
//            Thread.sleep(800);
//
//            moveForward(0.5, 1250);
//
//            turnLeft(0.5, 12);
//
//            moveForward(0.5, 4000);
//
//        }
//        else if(location == 2 )
//        {
//            robot.getWebcam().stopRecordingPipeline();
//            moveRight(0.5, 100);
//
//            moveForward(0.5, 400);
//
//            moveRight(0.5, 1250);
//
//            turnLeft(0.5, 180);
//
//            moveBackward(0.5, 350);
//            stopMotors();
//
//            topLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(0.5, 200);
//
//            moveRight(.5, 1700);
//
//            turnRight(.3, 45);
//
//            moveForward(.2, 350);
//
//            moveForward(0.13);
//            robot.getDuck().setPower(-0.3);
//            Thread.sleep(4000);
//            stopMotors();
//
//            robot.getDuck().setPower(-.65);
//            Thread.sleep(1000);
//            robot.getDuck().setPower(0);
//
//            moveBackward(0.5, 300);
//
//            turnLeft(0.5, 140);
//
//            moveBackward(0.5);
//            Thread.sleep(600);
//
//            moveBackward(0.2);
//            Thread.sleep(800);
//
//            moveForward(0.5, 1250);
//
//            turnLeft(0.5, 12);
//
//            moveForward(0.5, 4000);
//
//            robot.getBucket().setPosition(.3);
//
//        }
//    }
//}
//
