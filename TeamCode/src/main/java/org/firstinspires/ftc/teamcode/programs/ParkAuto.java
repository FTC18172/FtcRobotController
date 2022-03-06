package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;


@Autonomous(name = "ParkAuto", group = "OpModes")
public class ParkAuto extends UpliftAutoImpl
{
    public void body() throws InterruptedException
    {
        moveForward(0.5);
        robot.safeSleep(2500);
    }
}
