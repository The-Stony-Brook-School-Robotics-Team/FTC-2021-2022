package org.sbs.bears.robotframework.controllers;


import android.text.method.TextKeyListener;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideHeightController {

    Servo worker;

    public SlideHeightController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        //worker = hardwareMap.get(Servo.class, "slideLifter"); // TODO Make on the robot
    }

    public void setSlideHeight(SlideHeight height)
    {
        /*switch(height)
        {
            case ONE:
                worker.setPosition(slideHeightONE);
            case TWO:
                worker.setPosition(slideHeightTWO);
            case THREE:
                worker.setPosition(slideHeightTHREE);
            case FOUR_CAP:
                worker.setPosition(slideHeightFOUR);
            case ZERO_FLOOR:
                worker.setPosition(slideHeightZERO);
        }*/

    }

    // TODO configure
    public static double slideHeightZERO = 0;
    public static double slideHeightONE = 0;
    public static double slideHeightTWO = 0;
    public static double slideHeightTHREE = 0;
    public static double slideHeightFOUR = 0;

}
