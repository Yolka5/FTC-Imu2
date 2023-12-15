package org.firstinspires.ftc.teamcode.drive.opmode;



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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;


@Autonomous(group = "drive")
public class autoTest extends LinearOpMode {
    public static double SPEED = 0.15;
    boolean Starting = false;
    int ConusIndex = 0;
    double value;

    public static double FirstDistance = 48;
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
    private DcMotor elevator;
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
        elevator = hardwareMap.dcMotor.get("elevator");
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
                .forward(FirstDistance)
                .build();

        Trajectory trajectoryBackFirst = drive.trajectoryBuilder(new Pose2d())
                .back(-6)
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
                .strafeLeft(35)
                .build();

        TrajectorySequence trajectorySignalOneThree = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(32)
                .build();

        TrajectorySequence trajectoryTurnThird = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-53))
                .build();
        TrajectorySequence trajectoryTurnConus = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-50))
                .build();
        TrajectorySequence trajectoryTurnConus2 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(45))
                .forward(16.5)
                .build();
        TrajectorySequence trajectoryTurnConus1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(50))
                .back(-6)
                .build();

        Trajectory trajectoryForwardStart = drive.trajectoryBuilder(new Pose2d())
                .forward(-6)
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

                drive.followTrajectorySequence(trajectoryTurnConus);

                Starting = true;
                Parking = true;
                // follow red line once
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