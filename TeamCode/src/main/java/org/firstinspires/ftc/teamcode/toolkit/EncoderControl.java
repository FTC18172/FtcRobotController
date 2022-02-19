package org.firstinspires.ftc.teamcode.toolkit;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.UpliftRobot;

public class EncoderControl  {


    //    double integralSum = 0;
//    double P = 0.003;
//    double I = 0;
//    double D = 0;
//
//    ElapsedTime timer = new ElapsedTime();
//
//    double lastError = 0;
//
//    public  double PIDSetPosition(double reference, double state)
//    {
//        double error = reference - state;
//        integralSum += error * timer.seconds();
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        timer.reset();
//
//        double output = (error * P) + (derivative * D) + (integralSum * I);
//        return output;
//    }

}
