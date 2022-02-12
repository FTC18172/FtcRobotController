package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class UpliftAuto extends LinearOpMode {
    protected final UpliftRobot robot;

    protected abstract void initHardware();

    protected abstract void initAction();

    protected abstract void body() throws InterruptedException;

    protected abstract void exit() throws InterruptedException;

    protected abstract void  stopMotors();

    protected abstract void topLayer() throws InterruptedException;

    public UpliftAuto(UpliftRobot robot) {
        this.robot = robot;
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