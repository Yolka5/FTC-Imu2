package org.firstinspires.ftc.teamcode.drive.opmode;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(group = "drive")
public class finalAutoRight extends LinearOpMode {
    public static double SPEED = 0.15;
    boolean Starting = false;
    int ConusIndex = 0;
    double value;

    public static double FirstDistance = 46.5;
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
    Pose2d myPose = new Pose2d(-36, -60, Math.toRadians(-90));
    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));
    Pose2d EndPose = new Pose2d(39, -19, Math.toRadians(13.3));


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
        double ArmIntoCone = 0.17;
        double ArmIntake = 1;
        double ArmSecondFloor = 0.91;
        double ArmThirdfloor = 0.84;
        double ArmForthFloor = 0.79;
        double ArmThithFloor = 0.76;
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
        AngleServo.setPosition(Angle);
        Claw.setPosition(ClawPos);




        waitForStart();

        if (isStopRequested()) return;
//--------------------------------------------------------------------------------------
        Trajectory trajectoryForwardFirst = drive.trajectoryBuilder(new Pose2d())
                .back(FirstDistance)
                .build();

        Trajectory trajectoryBackFromParkingConus = drive.trajectoryBuilder(new Pose2d())
                .forward(5.3)
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

        TrajectorySequence trajectorySignalOne = drive.trajectorySequenceBuilder(EndPose)
                .strafeRight(28)
                .build();

        TrajectorySequence trajectorySignalOneThree = drive.trajectorySequenceBuilder(EndPose)
                .strafeLeft(32)
                .build();

        TrajectorySequence trajectoryTurnThird = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-53))
                .build();
        TrajectorySequence trajectoryTurnConus = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(45))
                .build();
        TrajectorySequence trajectoryTurnConus2 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(45))
                .forward(16.5)
                .build();
        TrajectorySequence trajectoryTurnConus1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(5)
                .turn(Math.toRadians(-45))
                .forward(16.8)
                .build();
        TrajectorySequence getToJunction = drive.trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                .forward(45)
                .lineToLinearHeading(new Pose2d(36.5, -21.5, Math.toRadians(-7)))
                .lineToLinearHeading(EndPose)
                .build();
        TrajectorySequence getToPark = drive.trajectorySequenceBuilder(EndPose)
                .lineToLinearHeading(new Pose2d(34, -36, Math.toRadians(-108)))
                .build();


        Trajectory trajectoryForwardStart = drive.trajectoryBuilder(new Pose2d(-40, -5, Math.toRadians(-180)))
                .back(5)
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

                orangeCone.setPosition(1);
                drive.setPoseEstimate(startPose);
                drive.followTrajectorySequence(getToJunction);
                //ele up


                //slidevout
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Claw.setPosition(0);
                Armservo.setPosition(ArmThithFloor);
                sleep(200);
                slider.setPower(0.8);
                slider.setTargetPosition(-495);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(500);
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(250);
                Armservo.setPosition(ArmIntoCone);
                sleep(250);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator down

                //arm down
                sleep(1100);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




                //second cycle
                orangeCone.setPosition(1);
//ele up
                //ele up
                //ele up


                //slidevout
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Claw.setPosition(0);
                sleep(100);
                Armservo.setPosition(ArmForthFloor);
                slider.setPower(0.9);
                slider.setTargetPosition(-495);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(500);
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(250);
                Armservo.setPosition(ArmIntoCone);
                sleep(250);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator down

                //arm down
                sleep(1100);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




                //third cycle
                orangeCone.setPosition(1);
                //ele up
                //ele up


                //slidevout
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Claw.setPosition(0);
                sleep(100);
                Armservo.setPosition(ArmThirdfloor);
                slider.setPower(0.9);
                slider.setTargetPosition(-495);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(500);
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(250);
                Armservo.setPosition(ArmIntoCone);
                sleep(250);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator down

                //arm down
                sleep(1100);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





                //forth cycle
                orangeCone.setPosition(1);
                //ele up
                //ele up


                //slidevout
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Claw.setPosition(0);
                sleep(200);
                Armservo.setPosition(ArmSecondFloor);
                sleep(250);
                slider.setPower(0.5);
                slider.setTargetPosition(-495);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(500);
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(250);
                Armservo.setPosition(ArmIntoCone);
                sleep(250);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator down

                //arm down
                sleep(1100);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






                //thith cycle
                //ele up
                //ele up


                //slidevout
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Claw.setPosition(0);
                sleep(100);
                Armservo.setPosition(ArmIntake);
                sleep(250);
                slider.setPower(0.5);
                slider.setTargetPosition(-495);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setTargetPosition(-900);
                elevator.setPower(1);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //cone deposit
                sleep(400);
                orangeCone.setPosition(0);

                //ele down and close visher
                sleep(700);
                orangeCone.setPosition(1);
                sleep(500);
                ClawPos = 0.65;
                Claw.setPosition(ClawPos);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
                elevator.setTargetPosition(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(250);
                Armservo.setPosition(ArmIntoCone);
                sleep(250);
                slider.setPower(1);
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator down

                //arm down
                sleep(1100);
                ClawPos = 0.2;
                Claw.setPosition(ClawPos);
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);






                //drop
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
                Armservo.setPosition(0.17);
                drive.setPoseEstimate(EndPose);
                drive.followTrajectorySequence(getToPark);
                Starting = true;
                Parking = true;
            }
            // parking
            else if (Parking) {
                if (wantedId == 1) {
                    drive.setPoseEstimate(EndPose);
                    drive.followTrajectorySequence(trajectorySignalOne);
                    Parking = false;
                }
                else if (wantedId == 2) {
                    //drop last cone
                    Parking = false;
                }
                else if (wantedId == 3) {
                    drive.setPoseEstimate(EndPose);
                    drive.followTrajectorySequence(trajectorySignalOneThree);
                    Parking = false;
                }

            }




//




        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}