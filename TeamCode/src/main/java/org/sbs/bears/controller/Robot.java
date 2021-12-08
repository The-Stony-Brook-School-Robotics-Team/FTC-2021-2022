package org.sbs.bears.controller;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public ColorStripController colorCtrl;
    public Robot(HardwareMap hwMap, Telemetry telemetry) {
        this.colorCtrl = new ColorStripController(hwMap);
    }
}
