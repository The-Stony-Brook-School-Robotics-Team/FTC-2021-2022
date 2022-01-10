package org.sbs.bears.robotframework.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorStripController {

    RevBlinkinLedDriver colorstrip2;
    RevSPARKMini brightLights;

    public ColorStripController(HardwareMap hardwareMap, Telemetry telemetry)
    {

        brightLights = hardwareMap.get(RevSPARKMini.class,"vout");
        colorstrip2 = hardwareMap.get(RevBlinkinLedDriver.class,"rgb");

    }
    public void setColorBright()
    {
        //brightLights;
    }
    public void setColor()
    {
        colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }
}
