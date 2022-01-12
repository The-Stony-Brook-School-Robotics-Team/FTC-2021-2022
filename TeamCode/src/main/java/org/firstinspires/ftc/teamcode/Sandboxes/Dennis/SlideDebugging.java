package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.SlideController;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "T - Slide Debugger", group = "default")
public class SlideDebugging extends LinearOpMode {

    private static int limitMin = 0;
    private static double PosLimitThreshold = 0.05;
    private static double NegLimitThreshold = -0.05;
    private SlideController slideController;

    @Override
    public void runOpMode() throws InterruptedException {
        slideController = new SlideController(hardwareMap, telemetry);

        waitForStart();

        while(!isStopRequested()) {
            limitMin = slideController.getMotorPosition();
            if(gamepad1.right_stick_x != 0 && gamepad1.right_stick_x > PosLimitThreshold && gamepad1.right_stick_x < NegLimitThreshold) {
                if(slideController.getMotorPosition() > limitMin) {
                    slideController.setMotorIncrement((int)gamepad1.right_stick_x);
                }

            }

            telemetry.addData("Slide Encoder Pos: ", slideController.getMotorPosition());
            telemetry.update();
        }



    }






}
