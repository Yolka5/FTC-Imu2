package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class distanceCheck extends LinearOpMode {
    public static double SPEED = 0.15;
    OpticalDistanceSensor lightSensor;
    private ColorSensor colorSensor;
    Boolean RedFirstTime = false;
    public double correction;
    private DistanceSensor distanceSensor;



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceJ");

        waitForStart();

        if (isStopRequested()) return;

        //  Trajectory trajectoryRight = drive.trajectoryBuilder(new Pose2d())

        // .build();
        // Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
        //.back(DISTANCE)

        //.build();
        //Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d())
        //.strafeLeft(DISTANCE)
        //.build();
        waitForStart();
//        drive.followTrajectory(trajectoryForward);

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("distance:", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();


//            drive.followTrajectory(trajectoryForward);
//            //drive.followTrajectory(trajectoryRight);
//            //drive.followTrajectory(trajectoryBackward);

//            //drive.followTrajectory(trajectoryLeft);




        }
    }}