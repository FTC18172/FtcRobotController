package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class UpliftAuto extends LinearOpMode {
    protected final UpliftRobot robot;

    protected abstract void initHardware();

    protected abstract void initAction();

    protected abstract void body() throws InterruptedException;

    protected abstract void exit() throws InterruptedException;

    public UpliftAuto() {
        this.robot = new UpliftRobot(this);
    }

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