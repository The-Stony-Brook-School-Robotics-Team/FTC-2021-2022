package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;

//@TeleOp(name="slide tester teleop")
public class NewSlideControllerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean qA = false,qX = false,qB = false, qUp = false,  qDown = false;
        SlideController ctrl = new SlideController(hardwareMap,telemetry);
        ctrl.initTeleop();
        waitForStart();
        new Thread(()->{
            while(opModeIsActive() && !isStopRequested())
            {
                telemetry.addData("Pos",ctrl.slideMotor.getCurrentPosition());
                telemetry.addData("pos2",ctrl.verticalServo.getPosition());
                telemetry.addData("state",ctrl.slideState.toString());
                telemetry.update();
            }
        }).start();
        ctrl.targetParams = SlideTarget.TOP_CAROUSEL;
        while(opModeIsActive() && !isStopRequested())
        {

            if(gamepad1.a)
            {
                qA = true;
                ctrl.incrementVerticalServo(0.1);
                // ctrl.incrementEncoderPosition(100);
                Log.d("SlideTester","finished extension");
                //telemetry.addData("pressed", 1);
                //telemetry.update();
            }
            if(qA && !gamepad1.a)
            {
                qA = false;
            }
            if(gamepad1.b)
            {
                qB = true;
                ctrl.incrementVerticalServo(-0.1);
            }
            if(qB && !gamepad1.b)
            {
                qB = false;
            }
            if(gamepad1.dpad_up && !qUp)
            {
                qUp = true;
                ctrl.incrementVerticalServo(.1);
                Log.d("SlideTester","increment servo");
            }
            if(qUp && !gamepad1.dpad_up)
            {
                qUp = false;
            }
            if(!qDown && gamepad1.dpad_down)
            {
                qDown = true;
                ctrl.incrementVerticalServo(-.1);
            }
            if(qDown && !gamepad1.dpad_down)
            {
                qDown = false;
            }

        }
    }
}
