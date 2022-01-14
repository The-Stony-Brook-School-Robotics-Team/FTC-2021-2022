package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;

@TeleOp(name="slide tester teleop")
public class NewSlideControllerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean qA = false,qX = false,qB = false, qUp = false,  qDown = false;
        SlideController ctrl = new SlideController(hardwareMap,telemetry);
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
        ctrl.targetParams = SlideTarget.THREE_CAROUSEL;
        while(opModeIsActive() && !isStopRequested())
        {

            if(!qA && gamepad1.a)
            {
                qA = true;
                ctrl.incrementEncoderPosition(200);
                Log.d("SlideTester","finished extension");
                //telemetry.addData("pressed", 1);
                //telemetry.update();
            }
            if(qA && !gamepad1.a)
            {
                qA = false;
            }
            if(!qB && gamepad1.b)
            {
                qB = true;
                ctrl.retractSlide();
            }
            if(qB && !gamepad1.b)
            {
                qB = false;
            }
            if(!qUp && gamepad1.dpad_up)
            {
                qUp = true;
                ctrl.incrementVerticalServo(.1);
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
