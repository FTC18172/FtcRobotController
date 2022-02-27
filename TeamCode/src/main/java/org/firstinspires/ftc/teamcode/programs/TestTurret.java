package org.firstinspires.ftc.teamcode.programs;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.PIDControl;

@Autonomous(name = "TestTurret", group = "OpModes")
public class TestTurret extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {

        if (robot.getPotentiometer().getVoltage() < 1.43) {
            while (robot.getPotentiometer().getVoltage() < 1.43) {
                robot.getTurret().setPower(0.1);
            }
            robot.getTurret().setPower(0);
            if (robot.getPotentiometer().getVoltage() > 1.43) {
                while (robot.getPotentiometer().getVoltage() > 1.43) {
                    robot.getTurret().setPower(-0.1);
                }
                robot.getTurret().setPower(0);
            }
        } else if (robot.getPotentiometer().getVoltage() > 1.43) {
            while (robot.getPotentiometer().getVoltage() > 1.43) {
                robot.getTurret().setPower(-0.1);
            }
            robot.getTurret().setPower(0);
        }

    }

}





