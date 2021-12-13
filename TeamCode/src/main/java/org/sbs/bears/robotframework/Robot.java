package org.sbs.bears.robotframework;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;

public class Robot {
    protected OpenCVController CVctrl;
    protected RoadRunnerController RRctrl;
    // TODO add other controllers here.
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) {
        this.CVctrl = new OpenCVController(hardwareMap,telemetry,mode);
        this.RRctrl = new RoadRunnerController(hardwareMap,telemetry);
    }
}
