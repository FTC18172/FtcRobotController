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
//    DcMotor leftFront, rightFront, leftBack, rightBack;
//    DcMotor intake, duck;
    DcMotor turret;
//    Servo bucket, arm, capX, capY, capArm;
//    CRServo cap;
    AnalogInput potentiometer;

    ColorSensor bucketSensor;
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public double worldX = 0, worldY = 0, rawAngle = 0, worldAngle = 0;
//    public BNO055IMU imu;
//    public OpenCvCamera webcam;
//    public FreightFrenzy pipeline;

    public UpliftRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        getHardware();
    }

    public void getHardware() {
        hardwareMap = opMode.hardwareMap;
//        initializeCamera();
        turret = hardwareMap.get(DcMotor.class, "turret");
//        leftFront = hardwareMap.get(DcMotor.class, "left_front");
//        rightFront = hardwareMap.get(DcMotor.class, "right_front");
//        leftBack = hardwareMap.get(DcMotor.class, "left_back");
//        rightBack = hardwareMap.get(DcMotor.class, "right_back");
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        duck = hardwareMap.get(DcMotor.class, "duck");
//        arm = hardwareMap.get(Servo.class, "arm");
//        bucket = hardwareMap.get(Servo.class, "bucket");
//        bucketSensor = hardwareMap.get(ColorSensor.class, "bucketSensor");
//        capX = hardwareMap.get(Servo.class, "capX");
//        capY = hardwareMap.get(Servo.class, "capY");
//        cap = hardwareMap.get(CRServo.class, "cap");
//        capArm = hardwareMap.get(Servo.class, "caparm");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");


        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        imu.initialize(parameters);


    }

//    public void initializeCamera() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                pipeline = new FreightFrenzy(opMode.telemetry);
//                webcam.setPipeline(pipeline);
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//    }


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

    public DcMotor getTurret() {
        return turret;
    }

    public AnalogInput getPotentiometer(){ return potentiometer;}

//    public Servo getBucket() {
//        return bucket;
//    }
//
//    public Servo getArm() {
//        return arm;
//    }
//
//    public DcMotor getLeftFront() {
//        return leftFront;
//    }
//
//    public DcMotor getLeftBack() {
//        return leftBack;
//    }
//
//    public DcMotor getRightBack() {
//        return rightBack;
//    }
//
//    public DcMotor getRightFront() {
//        return rightFront;
//    }

//    public OpenCvCamera getWebcam() {
//        return webcam;
//    }

//    public DcMotor getDuck() {
//        return duck;
//    }
//
//    public DcMotor getIntake() {
//        return intake;
//    }
//
//    public Servo getCapX() {
//        return capX;
//    }
//
//    public Servo getCapY() {
//        return capY;
//    }
//
//    public CRServo getCap() {
//        return cap;
//    }
//
//    public Servo getCapArm() {
//        return capArm;
//    }

//    public ColorSensor getBucketSensor() {
//        return bucketSensor;
//    }
}