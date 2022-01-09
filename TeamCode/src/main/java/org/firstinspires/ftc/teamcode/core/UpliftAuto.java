package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.vision.FreightFrenzy;
import org.openftc.easyopencv.OpenCvCamera;

public abstract class UpliftAuto extends LinearOpMode {
    UpliftRobot robot;

    public abstract void initHardware();

    public abstract void initAction();

    public abstract void body() throws InterruptedException;

    public abstract void exit() throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Initializing", "Started");
        telemetry.update();

        initHardware();
        initAction();

        telemetry.addData("Initializing", "Finished");
        telemetry.update();

        waitForStart();

        telemetry.addData("Body", "Started");
        telemetry.update();

        body();

        telemetry.addData("Body", "Finished");
        telemetry.update();

        exit();
    }

}