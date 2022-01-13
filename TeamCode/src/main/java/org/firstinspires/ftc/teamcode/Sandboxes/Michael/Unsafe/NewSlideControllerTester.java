package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class NewSlideControllerTester extends OpMode {
    private SlideController slide;
    @Override
    public void init() {
        slide = new SlideController(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if(gamepad1.a)slide.extendDropRetract(SlideTarget.TWO_CAROUSEL);
    }
}
