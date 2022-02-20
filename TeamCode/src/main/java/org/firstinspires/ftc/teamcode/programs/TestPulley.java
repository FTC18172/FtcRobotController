package org.firstinspires.ftc.teamcode.programs;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.UpliftAutoImpl;
import org.firstinspires.ftc.teamcode.core.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.PIDControl;

@Autonomous(name = "TestPulley", group = "OpModes")
public class TestPulley extends UpliftAutoImpl {

    @Override
    public void body() throws InterruptedException {

//        robot.getPulley().setPower(1);
//        while(opModeIsActive() && robot.getPulley().isBusy())
//        {
//            telemetry.addData("current position", robot.getPulley().getCurrentPosition() + "busy=" + robot.getPulley().isBusy());
//            telemetry.update();
//        }
        setPosition(0.1, 536);

    }

    public void setPosition(double power, int ticks)
    {
            robot.getPulley().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getPulley().setTargetPosition(ticks);
            robot.getPulley().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.getPulley().setPower(power);
            while(opModeIsActive() && robot.getPulley().isBusy())
            {
                telemetry.addData("current position", robot.getPulley().getCurrentPosition());
                telemetry.update();
            }
    }

}

//        PIDControl controller = new PIDControl();
//
//        double power = controller.PIDSetPosition(536, robot.getPulley().getCurrentPosition());
//        telemetry.addData("power", power);
//        telemetry.update();
//
//
//        while(power < -0.05 && power > 0.05){
//            robot.getPulley().setPower(power);
//            power = controller.PIDSetPosition(536, robot.getPulley().getCurrentPosition());
//            telemetry.addData("power", power);
//            telemetry.update();
//
//        }
//
//        robot.getPulley().setPower(0);
//
//    }
