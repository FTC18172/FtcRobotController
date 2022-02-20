package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;

@Autonomous(name = "TestMotor", group = "OpModes")
public class TestMotor extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {
        robot.getPulley().setPower(1);
        while (opModeIsActive() && robot.getPulley().isBusy())
        {
            telemetry.addData("current position", robot.getPulley().getCurrentPosition() + "busy=" + robot.getPulley().isBusy());
            telemetry.update();
        }
    }
}
