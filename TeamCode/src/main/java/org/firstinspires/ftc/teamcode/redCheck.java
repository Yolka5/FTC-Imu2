package org.firstinspires.ftc.teamcode;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class redCheck extends LinearOpMode {
    public static double SPEED = 0.15;
    OpticalDistanceSensor lightSensor;
    private ColorSensor colorSensor;
    Boolean RedFirstTime = false;
    public double correction;
    public double BlueCorrection;



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

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
            telemetry.addData("red:", colorSensor.red());
            telemetry.addData("correction:", correction);

            telemetry.addData("blue:", colorSensor.blue());
            telemetry.addData("blue correction:", BlueCorrection);
            telemetry.update();
            correction = (110 - colorSensor.red());
            BlueCorrection = (110 - colorSensor.blue());



//            drive.followTrajectory(trajectoryForward);
//            //drive.followTrajectory(trajectoryRight);
//            //drive.followTrajectory(trajectoryBackward);

//            //drive.followTrajectory(trajectoryLeft);




        }
    }}