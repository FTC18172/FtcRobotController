//package org.firstinspires.ftc.teamcode.programs;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//
//import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
//
//@Autonomous(name = "VisionPhubRed", group = "OpModes")
//public class VisionPhubRed extends UpliftAutoImpl {
//
//    @Override
//    public void body() throws InterruptedException {
//        int location = robot.pipeline.location;
//
//        if(location == 0 || location == -1 ){
//           robot.getWebcam().stopRecordingPipeline();
//
//           moveForward(0.5,500);
//
//           moveLeft(0.5,1000);
//
//           turnLeft(0.5,175);
//
//           moveBackward(0.5, 125);
//           stopMotors();
//
//           bottomLayer();
//
//           robot.getBucket().setPosition(0.5);
//
//           moveForward(0.5,150);
//
//           turnLeft(.5,100);
//
//           moveForward(.6,3000);
//        }
//        else if(location == 1){
//           robot.getWebcam().stopRecordingPipeline();
//
//           moveForward(0.5,500);
//
//           moveLeft(0.5,1000);
//
//           turnLeft(0.5,175);
//
//           moveBackward(0.5, 200);
//
//           stopMotors();
//
//           middleLayer();
//
//           robot.getBucket().setPosition(0.5);
//
//           moveForward(0.5,250);
//
//           turnLeft(.5,95);
//
//           moveForward(.6,3000);
//        }
//        else if(location == 2){
//            robot.getWebcam().stopRecordingPipeline();
//
//            moveForward(0.5,500);
//
//            moveLeft(0.5,1000);
//
//            turnLeft(0.5,180);
//
//            moveBackward(0.5, 420);
//            stopMotors();
//
//            topLayer();
//
//            robot.getBucket().setPosition(0.5);
//
//            moveForward(0.5,400);
//
//            turnLeft(.5,90);
//
//            moveForward(.6,3000);
//        }
//        robot.getBucket().setPosition(.3);
//    }
//}
//
