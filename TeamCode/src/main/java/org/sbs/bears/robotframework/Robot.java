package org.sbs.bears.robotframework;



import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.ColorStripController;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;

public class Robot {
    protected OpenCVController CVctrl;
    protected RoadRunnerController RRctrl;
    protected SlideController slideCtrl;
    protected IntakeControllerBlue IntakeCtrlBlue;
    protected IntakeControllerRed IntakeCtrlRed;
    protected ColorStripController colorCtrl;
    protected DuckCarouselController duckCtrl;
    // TODO add other controllers here.
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) {
        this.CVctrl = new OpenCVController(hardwareMap,telemetry,mode);
        this.RRctrl = new RoadRunnerController(hardwareMap,telemetry);
        this.slideCtrl = new SlideController(hardwareMap,telemetry);
        this.IntakeCtrlBlue = new IntakeControllerBlue(hardwareMap, slideCtrl.blueDumperServo, telemetry);
        this.IntakeCtrlRed = new IntakeControllerRed(hardwareMap, slideCtrl.redDumperServo, telemetry);
        //this.colorCtrl = new ColorStripController(hardwareMap, telemetry);
        this.duckCtrl = new DuckCarouselController(hardwareMap,telemetry);
    }
    public OpenCVController getCVctrl()
    {
        return CVctrl;
    }
    public RoadRunnerController getRRctrl()
    {
        return RRctrl;
    }
    public SlideController getSlideCtrl()
    {
        return slideCtrl;
    }
    public IntakeControllerBlue getIntakeCtrlBlue() {
        return IntakeCtrlBlue;
    }
    public IntakeControllerRed getIntakeCtrlRed() {
        return IntakeCtrlRed;
    }
    public ColorStripController getColorCtrl() {
        return colorCtrl;
    }
    public DuckCarouselController getDuckCtrl() {
        return duckCtrl;
    }

}
