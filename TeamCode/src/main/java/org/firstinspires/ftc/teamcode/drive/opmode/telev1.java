package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;


@TeleOp

public class telev1 extends LinearOpMode {
    private Servo Armservo;
    private Servo AngleServo;
    private Servo Claw;
    private Servo orangeCone;
    private Servo ConeLocker;
    private DcMotor elevator;
    private DcMotor slider;

    private TouchSensor magnetic1;
    private TouchSensor magnetic2;
    private DistanceSensor distanceSensor;
    private boolean toggleA;
    private  boolean resultA;
    private boolean toggleA1;
    private boolean resultA1;
    private boolean toggleB1;
    private boolean resultB1;


    private  boolean resultC2;
    private  boolean resultC3;
    private  boolean resultC5;
    private boolean toggleL;
    private boolean resultL;
    private  boolean resultC4;
    private boolean toggleX;
    private boolean toggleB;
    private  boolean resultX;
    private boolean toggleY;
    private boolean resultY;
    private  boolean resultB;
    private boolean resultLit;
    private boolean togglelitele;
    private boolean toggleOrange;
    private boolean resultOrange;
    private boolean toggleC5;
    private boolean toggleC4;
    private boolean toggleC3;
    private boolean toggleC2;
    private  boolean Gotmagnetic1;
    private  boolean Gotmagnetic2;
    double Angle = 0;
    double ClawPos = 0;
    boolean startElevator = false;
    double ArmIntoCone = 1;
    double ArmIntake = 0.3;
    double ArmSecondFloor = 0.43;
    double ArmThirdfloor = 0.47;
    double ArmForthFloor = 0.53;
    double ArmThithFloor = 0.56;



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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        magnetic1 = hardwareMap.get(TouchSensor.class, "frontMagnet");
        magnetic2 = hardwareMap.get(TouchSensor.class, "BackMagnet");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        toggleA = true;
        toggleB = true;
        toggleOrange = true;
        resultOrange = false;
        resultA = true;
        toggleX = true;
        toggleL = true;
        resultL = true;
        resultX = true;
        resultB = true;
        resultA = true;
        resultC2 = true;
        resultC3 = true;
        toggleB1 = true;
        resultB1 = true;
        resultA1 = true;
        toggleA1 = true;
        resultC5 = true;
        resultC4 = true;
        toggleC2 = true;
        toggleC3 = true;
        toggleC4 = true;
        toggleC5 = true;


        int elevatorH = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("distance ", distanceSensor.getDistance(DistanceUnit.CM));
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //MECHANISM CODE
            telemetry.addData("distance ", distanceSensor.getDistance(DistanceUnit.CM));

            if (gamepad2.right_bumper) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad2.left_stick_y * 0.28,
                                -gamepad2.left_stick_x * 0.28,
                                -gamepad2.right_stick_x * 0.28

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
            if(gamepad2.b)
            {
                while(distanceSensor.getDistance(DistanceUnit.CM) > 20)
                {
                    drive.setMotorPowers(0.15, -0.15, 0.15, -0.15);
                }
                while(distanceSensor.getDistance(DistanceUnit.CM) > 20)
                {
                    drive.setMotorPowers(-0.1, 0.1, -0.1, 0.1);
                }
                while(distanceSensor.getDistance(DistanceUnit.CM) > 20)
                {
                    drive.setMotorPowers(0.05, -0.05, 0.05, -0.05);
                }
                if(distanceSensor.getDistance(DistanceUnit.CM) > 13.7)
                {
                    while(distanceSensor.getDistance(DistanceUnit.CM) > 13.7)
                    {
                        drive.setMotorPowers(-0.1, -0.1, -0.1, -0.1);
                    }
                } else {
                    while(distanceSensor.getDistance(DistanceUnit.CM) < 13.7)
                    {
                        drive.setMotorPowers(0.1, 0.1, 0.1, 0.1);
                    }
                }
                drive.setMotorPowers(0, 0, 0, 0);
            }
            if(gamepad1.left_stick_button)
            {
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if( gamepad1.left_trigger > 0.5)
            {
                slider.setPower(1);
                slider.setTargetPosition(slider.getCurrentPosition() + 50);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (magnetic1.isPressed()) {
                telemetry.addData("magnet front: ", "True");
            }

            // left trigger lowering elevator



            // second floor
            if (gamepad1.dpad_up && !toggleC2) { // second floor
                toggleC2 = true;
                resultC2 = !resultC2;
                drive.setMotorPowers(0,0,0,0);
                if (!resultC2){
                    Armservo.setPosition(ArmSecondFloor);

                    slider.setPower(1);
                    slider.setTargetPosition(-500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(500);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(250);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(850);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setTargetPosition(0);
                    sleep(500);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
            } else if(!gamepad1.dpad_up && toggleC2) {
                toggleC2 = false;
            }
            // third floor
            else if (gamepad1.dpad_right && !toggleC3) { // third
                drive.setMotorPowers(0,0,0,0);
                toggleC3 = true;
                resultC3 = !resultC3;
                if (!resultC3){
                    Armservo.setPosition(ArmThirdfloor);

                    slider.setPower(1);
                    slider.setTargetPosition(-500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(500);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(250);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(850);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setTargetPosition(0);
                    sleep(500);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
            } else if(!gamepad1.dpad_right && toggleC3) {
                toggleC3 = false;
            }
            // fourth floor
            else if (gamepad1.dpad_down && !toggleC4) { // fourth
                drive.setMotorPowers(0,0,0,0);
                toggleC4 = true;
                resultC4 = !resultC4;
                if (!resultC4){
                    Armservo.setPosition(ArmForthFloor);

                    slider.setPower(1);
                    slider.setTargetPosition(-500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(500);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(250);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(850);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setTargetPosition(0);
                    sleep(500);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
            }
            else if(!gamepad1.dpad_down && toggleC4) {
                toggleC4 = false;
            }
            // fifth floor
            else if (gamepad1.dpad_left && !toggleC5) { // fifth
                drive.setMotorPowers(0,0,0,0);
                toggleC5 = true;
                resultC5 = !resultC5;
                if (!resultC5){
                    Armservo.setPosition(ArmThithFloor);

                    slider.setPower(1);
                    slider.setTargetPosition(-500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(500);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(250);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(850);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setTargetPosition(0);
                    sleep(500);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }
            } else if(!gamepad1.dpad_left && toggleC5) {
                toggleC5 = false;
            }
            else if (gamepad1.right_bumper && !toggleX) { // Cone locker
                drive.setMotorPowers(0,0,0,0);
                toggleX = true;
                resultX = !resultX;
                if (!resultX){
                    ConeLocker.setPosition(0.2);
                }

                else{
                    ConeLocker.setPosition(0.5);
                }
            } else if(!gamepad1.right_bumper && toggleX) {
                toggleX = false;
            }
            if(gamepad1.left_bumper&&!toggleL){
                drive.setMotorPowers(0,0,0,0);
                toggleL = true;
                resultL = !resultL;
                if (!resultL){
                    Armservo.setPosition(ArmIntake);
                    sleep(400);
                    slider.setPower(1);
                    slider.setTargetPosition(-200);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(500);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(250);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(850);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setTargetPosition(0);
                    sleep(500);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            else if(!gamepad1.left_bumper&&toggleL){
                toggleL = false;
            }



            // pressing X on controller
            if(gamepad1.x) {
                drive.setMotorPowers(0,0,0,0);
                elevator.setTargetPosition(-1800);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                orangeCone.setPosition(1);


                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(400);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            // pressing X on controller
            if(gamepad1.b) {
                drive.setMotorPowers(0,0,0,0);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                orangeCone.setPosition(1);


                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(400);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // pressing A on controller
            if(gamepad1.a&&!toggleA){
                drive.setMotorPowers(0,0,0,0);
                toggleA = true;
                resultA = !resultA;
                if (!resultA){
                    Armservo.setPosition(ArmIntake);
                    sleep(400);
                    slider.setPower(1);
                    slider.setTargetPosition(-500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(500);
                    Armservo.setPosition(ArmIntoCone);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(1050);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            else if(!gamepad1.a&&toggleA){
                toggleA = false;
            }

            if(gamepad2.y&&!toggleY){
                drive.setMotorPowers(0,0,0,0);
                toggleY = true;
                resultY = !resultY;
                if (!resultY){
                    Armservo.setPosition(ArmIntake);
                }

                else{
                    Armservo.setPosition(ArmIntoCone);
                }
            }

            else if(!gamepad2.y&&toggleY){
                toggleY = false;
            }
            if(gamepad2.x&&!toggleA1){
                drive.setMotorPowers(0,0,0,0);
                toggleA1 = true;
                resultA1 = !resultA1;
                if (!resultA1){
                    Armservo.setPosition(ArmIntake);
                    sleep(500);
                    slider.setPower(1);
                    slider.setTargetPosition(-500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(700);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(400);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elevator down

                    //arm down
                    sleep(1000);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            else if(!gamepad2.x&&toggleA1){
                toggleA1 = false;
            }
            if(gamepad2.a&&!toggleB1){
                toggleB1 = true;
                resultB1 = !resultB1;
                if (!resultB1){
                    ClawPos = 0;
                    Claw.setPosition(ClawPos);
                }

                else{
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                }
            }

            else if(!gamepad2.a&&toggleB1){
                toggleB1 = false;
            }

            if (gamepad1.y) {
                drive.setMotorPowers(0,0,0,0);
                ConeLocker.setPosition(0.5);
                    sleep(150);
                    //ele up
                    elevator.setTargetPosition(-1800);
                    elevator.setPower(1);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    orangeCone.setPosition(1);


                //slidevout


                    //cone deposit
                    sleep(500);
                    orangeCone.setPosition(0);
                    //unlock cone
                    sleep(600);
                    Armservo.setPosition(ArmIntake);
                    sleep(300);
                    slider.setPower(1);
                    slider.setTargetPosition(-200);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //ele down and close visher
                    orangeCone.setPosition(1);
                    //elevator down
                    sleep(700);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);
                    elevator.setTargetPosition(0);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(300);
                //arm down
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    sleep(700);
                    Armservo.setPosition(ArmIntoCone);
                    sleep(300);
                    slider.setPower(1);
                    slider.setTargetPosition(0);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(900);
                    ClawPos = 0.2;
                    Claw.setPosition(ClawPos);
                    slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }

            //stuff for arm position control
            AngleServo.setPosition(Angle);
            Claw.setPosition(ClawPos);
            if(gamepad1.right_trigger == 1)
            {
                orangeCone.setPosition(0);
            }
            if(gamepad1.right_trigger == 0)
            {
                orangeCone.setPosition(1);
            }

            telemetry.update();
        }
            // pressing Y on controller

    




    }

}