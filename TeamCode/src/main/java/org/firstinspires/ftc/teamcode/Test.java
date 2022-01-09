package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.core.UpliftAuto;

@Autonomous(name = "Test", group = "OpModes")
public class Test extends UpliftAuto {
    UpliftRobot robot;

    @Override
    public void initHardware() {
        robot = new UpliftRobot(this);

    }

    @Override
    public void initAction() {

    }

    @Override
    public void body() throws InterruptedException {
        robot.safeSleep(5000);
    }

    @Override
    public void exit() throws InterruptedException {

    }
}