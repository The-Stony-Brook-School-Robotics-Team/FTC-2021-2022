package org.sbs.bears.robotframework;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideHeightController;

public class Robot {
    protected OpenCVController CVctrl;
    protected RoadRunnerController RRctrl;
    protected SlideHeightController SlideHCtrl;
    // TODO add other controllers here.
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) {
        this.CVctrl = new OpenCVController(hardwareMap,telemetry,mode);
        this.RRctrl = new RoadRunnerController(hardwareMap,telemetry);
    }
    public OpenCVController getCVctrl()
    {
        return CVctrl;
    }
    public RoadRunnerController getRRctrl()
    {
        return RRctrl;
    }
    public SlideHeightController getSlideHCtrl()
    {
        return SlideHCtrl;
    }

}
