package org.firstinspires.ftc.teamcode.programs;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.EncoderControl;

@Autonomous(name = "TestPulley", group = "OpModes")
public class TestPulley extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {

        encoderSetPosition(0.05, 100);

        //        PIDControl controller = new PIDControl();
//
//        robot.getPulley().setPower(controller.PIDSetPosition(134, robot.getPulley().getCurrentPosition()));

    }


}
