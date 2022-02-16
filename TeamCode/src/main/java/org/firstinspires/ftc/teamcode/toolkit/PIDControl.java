package org.firstinspires.ftc.teamcode.toolkit;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl  {

    double integralSum = 0;
    double P = 1;
    double I = 0.001;
    double D = 0.01;

    ElapsedTime timer = new ElapsedTime();

    double lastError = 0;

    public  double PIDSetPosition(double reference, double state)
    {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * P) + (derivative * D) + (integralSum * I);
        return output;
    }

}
