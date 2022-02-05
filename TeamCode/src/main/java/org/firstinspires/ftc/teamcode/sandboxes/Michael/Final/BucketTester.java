package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Config
//@TeleOp(name="B- BUCKET TESTING", group="Linear Opmode")
public class BucketTester extends LinearOpMode {
    private Servo bucket;
    private boolean qA = false;
    private boolean qB = false;

    public double dumperPosition_CLOSED = 0.33;  // remeasured on jan 31 at 16h08
    public double dumperPosition_READY = 0.53;
    public double dumperPosition_EJECT = 0.74;
    public double dumperPosition_RETRACTING = 0.05;

    public void runOpMode() throws InterruptedException {

        bucket = hardwareMap.get(Servo.class, "du");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_up && !qA){
                bucket.setPosition(bucket.getPosition() + .01);
                qA = true;
            }
            if(!gamepad1.dpad_up && qA){
                qA = false;
            }
            if(gamepad1.dpad_down && !qB){
                bucket.setPosition(bucket.getPosition() - .01);
                qB = true;
            }
            if(!gamepad1.dpad_down && qB){
                qB = false;
            }
            if(gamepad1.x)  bucket.setPosition(dumperPosition_CLOSED);
            if(gamepad1.y)  bucket.setPosition(dumperPosition_READY);
            if(gamepad1.a)  bucket.setPosition(dumperPosition_EJECT);
            if(gamepad1.b)  bucket.setPosition(dumperPosition_RETRACTING);


            telemetry.addData("position: ", bucket.getPosition());
            telemetry.update();
        }
    }
}
