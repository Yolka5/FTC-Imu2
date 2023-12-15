package org.firstinspires.ftc.teamcode.drive.opmode;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;


@Config
@Autonomous(group = "drive")
public class threeConeAuto extends LinearOpMode {
    public static double SPEED = 0.15;
    boolean Starting = false;
    int ConusIndex = 0;
    double value;

    public static double FirstDistance = 46;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    private Servo Armservo;
    private Servo AngleServo;
    private Servo Claw;
    private Servo orangeCone;
    private Servo ConeLocker;
    private DcMotor elevator;
    private DcMotor slider;

    private TouchSensor magnetic1;
    private TouchSensor magnetic2;
    private boolean toggle;
    private  boolean result;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    boolean Parking = false;
    int wantedId;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
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
        double armservoTarget = 0.5;
        double Angle = 0.4;
        double ClawPos = 0.65;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;

                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    wantedId = tagOfInterest.id;
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Armservo.setPosition(armservoTarget);
        AngleServo.setPosition(Angle);
        Claw.setPosition(ClawPos);




        waitForStart();

        if (isStopRequested()) return;
//--------------------------------------------------------------------------------------
        Trajectory trajectoryForwardFirst = drive.trajectoryBuilder(new Pose2d())
                .back(FirstDistance)
                .build();

        Trajectory trajectoryBackFromParkingConus = drive.trajectoryBuilder(new Pose2d())
                .forward(4)
                .build();
        TrajectorySequence StrafeToCone = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(15)
                .turn(Math.toRadians(22))
                .back(9)
                .build();
        TrajectorySequence StrafeToConeR = drive.trajectorySequenceBuilder(new Pose2d())
                .back(-9)
                .turn(Math.toRadians(-22))
                .forward(-14)
                .build();

        TrajectorySequence trajectoryBackFirst = drive.trajectorySequenceBuilder(new Pose2d())
                .back(-6)
                .forward(-8)
                .build();
        Trajectory trajectoryBackSecond = drive.trajectoryBuilder(new Pose2d())
                .back(3.5)
                .build();


        TrajectorySequence trajectoryTurnFirst = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-110))
                .build();

        Trajectory trajectoryForwardSecond = drive.trajectoryBuilder(new Pose2d())
                .forward(4.5)
                .build();

        TrajectorySequence trajectoryTurnSecond = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(53))
                .build();

        TrajectorySequence trajectorySignalOne = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(-35)
                .build();

        TrajectorySequence trajectorySignalOneThree = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(-32)
                .build();

        TrajectorySequence trajectoryTurnThird = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-53))
                .build();
        TrajectorySequence trajectoryTurnConus = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(5)
                .turn(Math.toRadians(-115))
                .build();
        TrajectorySequence trajectoryTurnConus2 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(45))
                .forward(16.5)
                .build();
        TrajectorySequence trajectoryTurnConus1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(5)
                .turn(Math.toRadians(50))
                .build();

        Trajectory trajectoryForwardStart = drive.trajectoryBuilder(new Pose2d())
                .back(4)
                .build();

        Trajectory trajectorySignalTwo = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        //--------------------------------------------------------------------------------------

        waitForStart();
//        drive.followTrajectory(trajectoryForward);

        //stuff for arm position control



        while (opModeIsActive() && !isStopRequested()) {




            //-----------------------------------------------------------------------------
            // starting movement

            if (!Starting) {
                armservoTarget = 0.36;
                Armservo.setPosition(armservoTarget);
                orangeCone.setPosition(0);
                drive.followTrajectory(trajectoryForwardFirst);
                drive.followTrajectorySequence(trajectoryTurnConus);
                ConeLocker.setPosition(0.2);
                sleep(150);
                //ele up
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                orangeCone.setPosition(0.3);


                //slidevout


                //cone deposit
                sleep(1200);
                orangeCone.setPosition(1);
                //unlock cone
                sleep(500);
                ConeLocker.setPosition(0.5);
                sleep(300);
                orangeCone.setPosition(0);
                //elevator down
                sleep(700);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(StrafeToCone);
                armservoTarget = 0.866;
                Armservo.setPosition(armservoTarget);
                ClawPos = 0;
                Claw.setPosition(ClawPos);
                slider.setPower(1);
                slider.setTargetPosition(175);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //ele down and close visher
                sleep(1000);

                //arm down
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                sleep(500);



                Armservo.setPosition(armservoTarget);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armservoTarget = 0.4;
                Armservo.setPosition(armservoTarget);
                sleep(1200);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                 ///new cycle
                sleep(500);
                drive.followTrajectorySequence(StrafeToConeR);

                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                orangeCone.setPosition(0.3);


                //slidevout


                //cone deposit
                sleep(1200);
                orangeCone.setPosition(1);
                //unlock cone
                sleep(500);
                ConeLocker.setPosition(0.5);
                sleep(300);
                orangeCone.setPosition(0);
                //elevator down
                sleep(700);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.followTrajectorySequence(StrafeToCone);
                armservoTarget = 0.9;
                Armservo.setPosition(armservoTarget);
                ClawPos = 0;
                Claw.setPosition(ClawPos);
                slider.setPower(1);
                slider.setTargetPosition(175);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //ele down and close visher
                sleep(1000);

                //arm down
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                sleep(500);



                Armservo.setPosition(armservoTarget);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armservoTarget = 0.4;
                Armservo.setPosition(armservoTarget);
                sleep(1200);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                ///new cycle
                drive.followTrajectorySequence(StrafeToConeR);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                orangeCone.setPosition(0.3);


                //slidevout


                //cone deposit
                sleep(1200);
                orangeCone.setPosition(1);
//                drive.followTrajectory(trajectoryForwardStart);
//                elevator.setPower(1);
//                elevator.setTargetPosition(1800);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                orangeCone.setPosition(0);
//                sleep(3000);
//                orangeCone.setPosition(1);
//                sleep(1500);
//                orangeCone.setPosition(0);
//                drive.followTrajectorySequence(trajectoryTurnConus1);
//                elevator.setPower(1);
//                elevator.setTargetPosition(0);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                drive.followTrajectorySequence(trajectoryBackFirst);
//
                Starting = true;
                Parking = true;
                // follow red line once
                telemetry.update();

            }
            // parking
            else if (Parking) {
                if (wantedId == 1) {
                    drive.followTrajectorySequence(trajectorySignalOne);
                    Parking = false;
                }
                else if (wantedId == 2) {
                    Parking = false;
                }
                else if (wantedId == 3) {
                    drive.followTrajectorySequence(trajectorySignalOneThree);
                    Parking = false;
                }

            }




//


            telemetry.update();


        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.update();

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}