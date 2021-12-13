package org.sbs.bears.robotframework;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    protected OpenCVController CVctrl;
    protected RoadRunnerController RRctrl;
    // TODO add other controllers here.
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.CVctrl = new OpenCVController(hardwareMap,telemetry);
        this.RRctrl = new RoadRunnerController(hardwareMap,telemetry);
    }
}
