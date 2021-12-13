package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideHeightController;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

public class AutonomousBrain {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideHeightController slideHCtrl;




    AutonomousStates majorState = AutonomousStates.STOPPED;
    AutonomousBackForthSubStates minorState = AutonomousBackForthSubStates.STOPPED;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;



    enum AutonomousStates {
        STOPPED,
        ONE_READ_DUCK,
        TWO_SET_SLIDE_HEIGHT,
        THREE_DEPOSIT_BOX,
        THREE_OPT_CAROUSEL,
        FOUR_DRIVE_TO_WAREHOUSE,
        FOUR_SPLINE_THROUGH_WAREHOUSE,
        FOUR_B_SET_SLIDE_HEIGHT_3,
        FIVE_BACK_FORTH,
        SIX_PARKING_WAREHOUSE,
        FINISHED
    }
    enum AutonomousBackForthSubStates {
        STOPPED,
        ONE_INTAKE,
        TWO_FORWARD,
        THREE_SLIDE_OUT,
        FOUR_SLIDE_IN,
        FIVE_BACKWARD
    }

    public AutonomousBrain(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode)
    {
        this.mode = mode;
        this.robot = new Robot(hardwareMap,telemetry,mode);
        this.CVctrl = robot.getCVctrl();
        this.RRctrl = robot.getRRctrl();
        this.slideHCtrl = robot.getSlideHCtrl();
    }
    public void doAutonAction()
    {
        switch(majorState)
        {
            case STOPPED:
                majorState = AutonomousStates.ONE_READ_DUCK;
                return;
            case ONE_READ_DUCK:
                heightFromDuck = CVctrl.getWhichTowerHeight();
                majorState = AutonomousStates.TWO_SET_SLIDE_HEIGHT;
                return;

        }
    }
}
