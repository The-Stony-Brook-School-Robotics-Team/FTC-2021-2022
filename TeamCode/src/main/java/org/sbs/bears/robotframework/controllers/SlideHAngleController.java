package org.sbs.bears.robotframework.controllers;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.enums.SlideHAngle;
import org.sbs.bears.robotframework.enums.SlideHeight;

public class SlideHAngleController {

    Servo worker;

    public SlideHAngleController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        worker = hardwareMap.get(Servo.class, "hz");
    }

    public void setSlideHAngle(SlideHAngle angle)
    {
        switch(angle)
        {
            case ZERO_STRAIGHT:
                worker.setPosition(slideAngleZERO);
                return;
            case FAR_L:
                worker.setPosition(slideAngleFAR_L);
                return;
            case FAR_R:
                worker.setPosition(slideAngleFAR_R);
                return;
            case MED_L:
                worker.setPosition(slideAngleMED_L);
                return;
            case MED_R:
                worker.setPosition(slideAngleMED_R);
                return;

        }

    }

    // TODO configure
    public static double slideAngleZERO = 0;
    public static double slideAngleFAR_L = 0;
    public static double slideAngleFAR_R = 0;
    public static double slideAngleMED_L = 0;
    public static double slideAngleMED_R = 0;


}
