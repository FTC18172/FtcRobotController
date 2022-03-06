package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.toolkit.vision.FreightFrenzy;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class UpliftRobot {
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor intake, turret, arm1, arm2;
    Servo tinyArm, cap, bucketLatch;
    AnalogInput potentiometer;

    ColorSensor bucketSensor, bottomSensor;
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public BNO055IMU imu;
    public OpenCvCamera webcam;
    public FreightFrenzy pipeline;

    public final double redTurretAngle = 2.42;
    public final double blueTurretAngle = 0.86;
    public final double turretAngleMid = 1.4;
    public final double rightAngle = 3.133;
    public final double leftAngle = 0.63;

    public final double openBucketLatch = 0.4;
    public final double bottomBucketLatch = 0.25;
    public final double closeBucketLatch = 0;

    public final double armTurretPos = 0.55;
    public final double armTopLayer = 0.7;
    public final double armMidLayer = 0.75;
    public final double armBottomLayer = 1;
    public final double armBasePos = 0.05;



    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;
        initializeCamera();
        turret = hardwareMap.get(DcMotor.class, "turret");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        bucketLatch = hardwareMap.get(Servo.class, "bucketLatch");
        bucketSensor = hardwareMap.get(ColorSensor.class, "bucketSensor");
        bottomSensor = hardwareMap.get(ColorSensor.class, "bottomSensor");


        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        tinyArm = hardwareMap.get(Servo.class, "tinyArm");

        cap = hardwareMap.get(Servo.class, "cap");




        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);


    }

    public void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                pipeline = new FreightFrenzy(opMode.telemetry);
                webcam.setPipeline(pipeline);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }


    public void safeSleep(int duration) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < duration) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public Servo getTinyArm()
    {
        return tinyArm;
    }

    public Servo getCap()
    {
        return cap;
    }

    public DcMotor getTurret() {
        return turret;
    }

    public AnalogInput getPotentiometer(){ return potentiometer;}

    public Servo getBucketLatch() {
        return bucketLatch;
    }

    public DcMotor getArm1() {
        return arm1;
    }
    public DcMotor getArm2() {
        return arm2;
    }

    public DcMotor getLeftFront() {
        return leftFront;
    }

    public DcMotor getLeftBack() {
        return leftBack;
    }

    public DcMotor getRightBack() {
        return rightBack;
    }

    public DcMotor getRightFront() {
        return rightFront;
    }

    public OpenCvCamera getWebcam() {
        return webcam;
    }

    public DcMotor getIntake() {
        return intake;
    }

    public ColorSensor getBucketSensor() {
        return bucketSensor;
    }

    public ColorSensor getBottomSensor()
    {
        return bottomSensor;
    }
}