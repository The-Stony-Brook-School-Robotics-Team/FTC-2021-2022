package org.sbs.bears.robotframework.controllers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Sleep;

public class ColorStripController {

    RevBlinkinLedDriver colorstrip2;
    DcMotor brightLights;

    public ColorStripController(HardwareMap hardwareMap, Telemetry telemetry)
    {

        colorstrip2 = hardwareMap.get(RevBlinkinLedDriver.class,"rgb");

    }
    public void setColorBright()
    {
        Sleep.sleep(500);
        Sleep.sleep(500);

    }
    public void setColor()
    {
        colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
    }

}
