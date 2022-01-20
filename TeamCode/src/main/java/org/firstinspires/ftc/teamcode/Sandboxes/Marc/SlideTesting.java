package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.sbs.bears.robotframework.controllers.SlideController;

@TeleOp(name = "AAAAA - COOL TELEOP TESTER")
public class SlideTesting extends LinearOpMode
{
    boolean pA = false, pUp = false, pDown = false;
    SlideController slideController;

    boolean slideOut = false;

    @Override
    public void runOpMode() throws InterruptedException {
        slideController = new SlideController(hardwareMap, telemetry);



        while(!isStopRequested()) {
            if(gamepad1.a && !pA) {
                if(slideOut) {
                    slideController.retractSlide();
                    slideOut = true;
                } else if(!slideOut) {
                    slideController.extendSlide();
                    slideOut = false;
                }
                pA = true;
            } else if(!gamepad1.a && pA) {
                pA = false;
            }

            if(gamepad1.right_stick_y < -0.02 || gamepad1.right_stick_y > -0.02) {
                double stickValue = gamepad1.right_stick_y * -0.2;
                slideController.incrementEncoderPosition((int) (stickValue * Configuration.DefaultSlideTicks));
                Log.d("SLIDE TESTER", "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            }

            if(gamepad1.dpad_up && !pUp) {
                slideController.incrementVerticalServo(0.02);
                Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                pUp = false;
            }

            if(gamepad1.dpad_down && !pDown) {
                slideController.incrementVerticalServo(-0.02);
                Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pDown = true;
            } else if(!gamepad1.dpad_down && pDown) {
                pDown = false;
            }

            telemetry.addData("Slide Ext",slideController.slideMotor.getCurrentPosition());
            telemetry.addData("Slide Angle",slideController.verticalServo.getPosition());
            telemetry.update();

        }



    }
}
