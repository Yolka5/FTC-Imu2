package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(group = "drive")
public class AutonomBlueLeft extends LinearOpMode {
    public static double SPEED = 0.15;
    OpticalDistanceSensor lightSensor;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    Boolean RedFirstTime = false;
    public double correction;
    public static double FirstDistance = 45.5;
    boolean Starting = false;
    boolean SawRed = false;
    boolean SawCancer = false;
    boolean SawCancer1 = false;
    int ConusIndex = 0;
    double value;
    private Servo Armservo;
    private Servo AngleServo;
    private Servo Claw;

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

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    boolean Parking = true;
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
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
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
                .back(6)
                .build();

        TrajectorySequence trajectoryTurnFirst = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-90))
                .build();

        Trajectory trajectoryForwardSecond = drive.trajectoryBuilder(new Pose2d())
                .forward(3.5)
                .build();
        Trajectory trajectoryBackSecond = drive.trajectoryBuilder(new Pose2d())
                .back(3.5)
                .build();

        TrajectorySequence trajectoryTurnSecond = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(55))
                .build();

        TrajectorySequence trajectorySignalOne = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(19)
                .build();
        TrajectorySequence trajectorySignalOneTwo = drive.trajectorySequenceBuilder(new Pose2d())
                .back(5)
                .build();
        TrajectorySequence trajectorySignalOneThree = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-150))

                .forward(23)
                .build();

        TrajectorySequence trajectoryTurnThird = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-55))
                .build();
        TrajectorySequence trajectoryTurnConus = drive.trajectorySequenceBuilder(new Pose2d())
                .back(16.5)
                .turn(Math.toRadians(-38))
                .build();
        TrajectorySequence trajectoryTurnConus2 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(45))
                .build();
        TrajectorySequence trajectoryTurnConus1 = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(170))
                .build();

        Trajectory trajectoryForwardStart = drive.trajectoryBuilder(new Pose2d())
                .forward(6)
                .build();
        TrajectorySequence trajectoryForwardConus = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(8.5)
                .build();
        TrajectorySequence trajectoryBackConus = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(-8.5)
                .strafeRight(-5)
                .build();


        Trajectory trajectoryback1 = drive.trajectoryBuilder(new Pose2d())
                .back(8)
                .build();
        Trajectory NoBack = drive.trajectoryBuilder(new Pose2d())
                .back(1)
                .build();

        //--------------------------------------------------------------------------------------

        waitForStart();
//        drive.followTrajectory(trajectoryForward);


        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("red:", colorSensor.red());
            telemetry.addData("correction:", correction);
            telemetry.addData("distance:", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            correction = (360 - colorSensor.red());
            correction = correction / 800;



            //-----------------------------------------------------------------------------
            // starting movement
            if (!Starting) {
                drive.followTrajectory(trajectoryForwardStart);
                drive.followTrajectorySequence(trajectoryTurnSecond);
                drive.followTrajectory(trajectoryForwardSecond);
                ClawPos = 0.0;
                Claw.setPosition(ClawPos);
                drive.followTrajectory(trajectoryBackSecond);
                drive.followTrajectorySequence(trajectoryTurnThird);
                armservoTarget = 0;
                Angle = 0.4;
                ClawPos = 0.65;
                Armservo.setPosition(armservoTarget);
                AngleServo.setPosition(Angle);
                Claw.setPosition(ClawPos);


                drive.followTrajectory(trajectoryForwardFirst);
                drive.followTrajectorySequence(trajectoryTurnFirst);
                Starting = true;
                // follow red line once
            } else if (!SawCancer) {
                if (Starting && !SawRed && colorSensor.blue() < 450) {
                    telemetry.addData("driving to red", "driving to red");
                    drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
                } else if (Starting && !SawRed && colorSensor.blue() >= 450) {
                    SawRed = true;
                    telemetry.addData("he saw red", "he saw red");
                } else if (Starting && SawRed && !SawCancer)
                    if(distanceSensor.getDistance(DistanceUnit.CM) > 34) {
                        if ((correction * 100) <= 0) {
                            drive.setMotorPowers(0.1 - correction, 0.1 - correction, 0.1, 0.1);
                        } else {
                            drive.setMotorPowers(0.1, 0.1, 0.1 + correction, 0.1 + correction);
                        }
                    }

                // take conus and puts on the little bar 2 times

                    else {
                        SawCancer = true;
                        drive.followTrajectory(trajectoryback1);
                        armservoTarget = 0.4;
                        Angle = 0.70;
                        Armservo.setPosition(armservoTarget);
                        AngleServo.setPosition(Angle);
                        Claw.setPosition(ClawPos);
                        sleep(500);

                        ClawPos = 0.65;
                        Armservo.setPosition(armservoTarget);
                        AngleServo.setPosition(Angle);
                        Claw.setPosition(ClawPos);
                        sleep(500);
                        armservoTarget = 0.5;
                        Angle = 0.4;
                    }
                // take conus and puts on the little bar 2 times
            } else if (SawCancer && Starting && SawRed && ConusIndex < 1)
            {
                telemetry.addData("distance:", distanceSensor.getDistance(DistanceUnit.CM));
                drive.followTrajectorySequence(trajectoryTurnConus);
                ClawPos = 0.65;
                Armservo.setPosition(armservoTarget);
                AngleServo.setPosition(Angle);
                Claw.setPosition(ClawPos);
                sleep(500);
                drive.followTrajectorySequence(trajectoryForwardConus);
                ClawPos = 0;
                Armservo.setPosition(armservoTarget);
                AngleServo.setPosition(Angle);
                Claw.setPosition(ClawPos);
                drive.followTrajectorySequence(trajectoryTurnConus2);
                distanceSensor.getDistance(DistanceUnit.CM);


                // follow line
//                if (distanceSensor.getDistance(DistanceUnit.CM) > 38) {
//                    while (distanceSensor.getDistance(DistanceUnit.CM) > 38) {
//                        if ((correction * 100) <= 0) {
//                            drive.setMotorPowers(0.1 - correction, 0.1 - correction, 0.1, 0.1);
//                        } else {
//                            drive.setMotorPowers(0.1, 0.1, 0.1 + correction, 0.1 + correction);
//                        }
//                    }
//                }

                ConusIndex += 1;
//                drive.followTrajectorySequence(trajectoryTurnConus1);
            }
            // parking
            else if (ConusIndex == 1 && SawCancer && SawRed && Starting && Parking) {
                if (wantedId == 1) {
                    drive.followTrajectorySequence(trajectorySignalOne);
                    Angle = 0.3;
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    AngleServo.setPosition(Angle);
                    Parking = false;
                }
                else if (wantedId == 2) {
                    drive.followTrajectorySequence(trajectorySignalOneTwo);
                    Angle = 0.3;
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    AngleServo.setPosition(Angle);
                    Parking = false;
                }
                else if (wantedId == 3) {
                    drive.followTrajectorySequence(trajectorySignalOneThree);
                    Angle = 0.3;
                    ClawPos = 0.65;
                    Claw.setPosition(ClawPos);
                    AngleServo.setPosition(Angle);
                    Parking = false;
                }

            }
            Armservo.setPosition(armservoTarget);
            AngleServo.setPosition(Angle);
            Claw.setPosition(ClawPos);




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