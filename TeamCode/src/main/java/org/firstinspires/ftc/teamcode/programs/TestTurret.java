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
//        }

        robot.getRightFront().setTargetPosition(1000);
        robot.getLeftFront().setTargetPosition(1000);
        robot.getRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightFront().setPower(0.5);
        robot.getLeftFront().setPower(0.5);
        while (opModeIsActive() && robot.getArm1().isBusy() && robot.getArm2().isBusy())
        {
            telemetry.addData("arm1 current position", robot.getRightFront().getCurrentPosition());
            telemetry.addData("arm2 current position", robot.getLeftFront().getCurrentPosition());
            telemetry.update();
        }    }

}





