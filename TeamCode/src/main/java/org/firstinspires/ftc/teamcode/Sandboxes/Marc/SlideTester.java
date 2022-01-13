package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.SlideController;

@TeleOp
public class SlideTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean qA = false,qX = false,qB = false;
        SlideController ctrl = new SlideController(hardwareMap,telemetry);
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            if(!qA && gamepad1.a)
            {
                qA = true;
                ctrl.extendSlide();
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

        }
    }
}
