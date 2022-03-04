package org.firstinspires.ftc.teamcode.programs;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;

@Autonomous(name = "TestTurret", group = "OpModes")
public class TestTurret extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {

//        if (robot.getPotentiometer().getVoltage() < 1.43) {
//            while (robot.getPotentiometer().getVoltage() < 1.43) {
//                robot.getTurret().setPower(0.1);
//            }
//            robot.getTurret().setPower(0);
//            if (robot.getPotentiometer().getVoltage() > 1.43) {
//                while (robot.getPotentiometer().getVoltage() > 1.43) {
//                    robot.getTurret().setPower(-0.1);
//                }
//                robot.getTurret().setPower(0);
//            }
//        } else if (robot.getPotentiometer().getVoltage() > 1.43) {
//            while (robot.getPotentiometer().getVoltage() > 1.43) {
//                robot.getTurret().setPower(-0.1);
//            }
//            robot.getTurret().setPower(0);


//        armSetPosition(0.3,1008);

        robot.getTinyArm().setPosition(0.5);
        robot.getBucketLatch().setPosition(0.5);
            }
        }





