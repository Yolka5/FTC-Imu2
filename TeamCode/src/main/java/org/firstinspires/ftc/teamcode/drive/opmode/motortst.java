package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
@Config

@Autonomous(group = "drive")
    //@Disabled
public class motortst extends LinearOpMode
{
    public static int ticksdc = 50;

    DcMotor armmotor;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {



        armmotor = hardwareMap.dcMotor.get("armmotor");

        telemetry.addData("Mode", "waiting");
        telemetry.update();
        armmotor.setTargetPosition(0);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for start button.

        waitForStart();


        while (opModeIsActive())
        {
            armmotor.setTargetPosition(ticksdc);

            armmotor.setPower(0.25);

            telemetry.update();
        }
    }
}