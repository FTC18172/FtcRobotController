package org.firstinspires.ftc.teamcode.programs;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;

@Autonomous(name = "StatesCycleRed", group = "OpModes")
public class StatesCycleRed extends UpliftAutoImpl {
    @Override
    public void body() throws InterruptedException {
        int location = robot.pipeline.location;
        if (location == 0 || location == -1) {
            robot.getWebcam().stopRecordingPipeline();

            //setBucketUp();
            blueTurretPos(redTurretAngle);
            //armSetPosition(1, 1008);
            //setBucketLow();
            //setBucketUp();
            blueTurretPos(turretPosMid);
            //setBucketDown();

        } else if (location == 1) {
            robot.getWebcam().stopRecordingPipeline();

            //setBucketUp();
            blueTurretPos(redTurretAngle);
            //armSetPosition(1, 1008);
            //setBucketMid();
            //setBucketUp();
            blueTurretPos(turretPosMid);
            //setBucketDown();

        } else if (location == 2) {
            robot.getWebcam().stopRecordingPipeline();

            //setBucketUp();
            blueTurretPos(redTurretAngle);
            //armSetPosition(1, 1008);
            //setBucketHigh();
            //setBucketUp();
            blueTurretPos(turretPosMid);
            //setBucketDown();

        }
//        for (int i = 0; i < 3; i++) {
//            while (robot.getBucketSensor().alpha() < 1000) {
//                moveForward(0.5);
//                robot.getIntake().setPower(0.6);
//                if (robot.getBucketSensor().alpha() > 1000) {
//                    stopMotors();
//                    break;
//                }
//            }
//            while (robot.getBottomSensor().alpha() < 1000) {
//                robot.getIntake().setPower(-0.5);
//                moveBackward(0.5);
//                if (robot.getBucketSensor().alpha() > 1000) {
//                    stopMotors();
//                    break;
                }
            }
//            moveBackward(0.5, 500);
//            //setBucketUp();
//            blueTurretPos(redTurretAngle);
//            //armSetPosition(1, 1008);
//            //setBucketHigh();
//            //setBucketUp();
//            blueTurretPos(turretPosMid);
//            //setBucketDown();
//        }
//        moveForward(1, 700);
//    }
//}
