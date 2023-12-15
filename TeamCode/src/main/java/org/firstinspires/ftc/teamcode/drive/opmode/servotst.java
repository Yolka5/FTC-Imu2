package org.firstinspires.ftc.teamcode.drive.opmode;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="srvo", group="Exercises")
//@Disabled
public class servotst extends LinearOpMode
{
    Servo   armServo;
    CRServo contServo;
    float   leftY, rightY;
    double  armPosition, gripPosition, contPower;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {



        armServo = hardwareMap.servo.get("arm");

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();


        while (opModeIsActive())
        {
            leftY = gamepad1.left_stick_y * -1;



            // check the gamepad buttons and if pressed, increment the appropriate position
            // variable to change the servo location.

            // move arm down on A button if not already at lowest position.


            // set the servo position/power values as we have computed them.
            armServo.setPosition(leftY);

            telemetry.addData("arm servo", "position=" + leftY + "  actual=" + armServo.getPosition());

            //telemetry.addData("arm servo", String.format("position=%.2f  actual=%.2f", armPosition,
            //    armServo.getPosition()));

            //telemetry.addData("grip servo", String.format("position=%.2f  actual=%.2f", gripPosition,
            //    gripServo.getPosition()));

            if (gamepad1.a) {
                armServo.setPosition(0);
            }

            //telemetry.addData("arm servo", "position=%.2f  actual=%.2f", armPosition, armServo.getPosition());

            //telemetry.addData("grip servo", "position=%.2f  actual=%.2f", gripPosition, gripServo.getPosition());

            telemetry.update();
        }
    }
}