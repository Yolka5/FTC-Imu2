package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;


@TeleOp

public class elevatorTest extends LinearOpMode {
    private Servo Armservo;
    private Servo AngleServo;
    private Servo Claw;
    private Servo orangeCone;
    private Servo ConeLocker;
    private DcMotor elevator;
    private DcMotor slider;

    private TouchSensor magnetic1;
    private TouchSensor magnetic2;
    private boolean toggleA;
    private  boolean resultA;
    private  boolean resultC2;
    private  boolean resultC3;
    private  boolean resultC5;
    private  boolean resultC4;
    private boolean toggleX;
    private boolean toggleB;
    private  boolean resultX;
    private  boolean resultB;
    private boolean resultLit;
    private boolean togglelitele;
    private boolean toggleOrange;
    private boolean resultOrange;
    private boolean toggleC5;
    private boolean toggleC4;
    private boolean toggleC3;
    private boolean toggleC2;
    private long newTime;
    private float timeEl;
    private  boolean Gotmagnetic1;
    private  boolean Gotmagnetic2;
    double armservoTarget = 0.5;
    double Angle = 0;
    double ClawPos = 0;
    private long eleTime = 0;
    private long firstTime = 0;
    boolean startElevator = false;


    @Override
    public void runOpMode() {
        Armservo = hardwareMap.get(Servo.class, "arm");
        AngleServo = hardwareMap.get(Servo.class, "arm angle");
        Claw = hardwareMap.get(Servo.class, "clips");
        orangeCone = hardwareMap.get(Servo.class, "orangeCone");
        ConeLocker = hardwareMap.get(Servo.class, "coneLocker");
        elevator = hardwareMap.dcMotor.get("elevator");
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider = hardwareMap.dcMotor.get("leftEncoder");
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magnetic1 = hardwareMap.get(TouchSensor.class, "frontMagnet");
        magnetic2 = hardwareMap.get(TouchSensor.class, "BackMagnet");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        toggleA = false;
        toggleB = true;
        toggleOrange = true;
        resultOrange = false;
        resultA = true;
        toggleX = true;
        resultX = true;
        resultB = true;
        resultA = true;
        resultC2 = true;
        resultC3 = true;
        resultC5 = true;
        resultC4 = true;
        toggleC2 = true;
        toggleC3 = true;
        toggleC4 = true;
        toggleC5 = true;


        int elevatorH = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //MECHANISM CODE
            newTime = eleTime - firstTime;
            if(gamepad2.y)
            {
                armservoTarget = 0.36;
            }
            if (gamepad2.right_bumper) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad2.left_stick_y * 0.35,
                                -gamepad2.left_stick_x * 0.35,
                                -gamepad2.right_stick_x * 0.35

                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad2.left_stick_y * 0.6,
                                -gamepad2.left_stick_x * 0.6,
                                -gamepad2.right_stick_x * 0.6

                        )
                );
            }


            drive.update();

            telemetry.addData("elevator position:", elevator.getCurrentPosition());
            telemetry.addData("elevator time:", newTime);
            telemetry.addData("slider time:", slider.getCurrentPosition());


            if (magnetic1.isPressed()) {
                telemetry.addData("magnet front: ", "True");
            }

            // left trigger lowering elevator
            if(gamepad1.y)
            {
                toggleA = true;
            }
            if(elevator.getCurrentPosition() <= -1790)
            {
                toggleA = false;
                firstTime = System.currentTimeMillis();
            }
            if(toggleA)
            {
                elevator.setTargetPosition(-1800);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                eleTime=System.currentTimeMillis();            }
            else{
                eleTime = eleTime;
            }
            //stuff for arm position control
            Armservo.setPosition(armservoTarget);
            AngleServo.setPosition(Angle);
            Claw.setPosition(ClawPos);
            orangeCone.setPosition(gamepad1.right_trigger);
            telemetry.update();
        }
        // pressing Y on controller






    }

}