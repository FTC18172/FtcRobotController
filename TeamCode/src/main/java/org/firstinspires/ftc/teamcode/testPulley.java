package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.toolkit.PIDControl;

@Autonomous(name = "TestPulley", group = "OpModes")
public class testPulley extends UpliftAutoImpl {


    @Override
    public void body() throws InterruptedException {
        PIDControl controller = new PIDControl();

        robot.getPulley().setPower(controller.PIDSetPosition(360, robot.getPulley().getCurrentPosition()));
    }


}
