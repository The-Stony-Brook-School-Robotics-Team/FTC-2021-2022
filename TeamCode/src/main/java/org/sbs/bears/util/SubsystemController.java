package org.sbs.bears.util;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubsystemController {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public SubsystemController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
    public abstract boolean shutDown();

}
