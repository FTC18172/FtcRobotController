package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class UpliftRobot {
    DcMotor lf, rf, lb, rb;
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;


    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();


    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;
        lf = hardwareMap.get(DcMotor.class, "left_front");
        lb = hardwareMap.get(DcMotor.class, "left_back");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        rb = hardwareMap.get(DcMotor.class, "right_back");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    }
