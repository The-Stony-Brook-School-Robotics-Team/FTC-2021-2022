package org.firstinspires.ftc.teamcode.common.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideExtensionController;
import org.sbs.bears.robotframework.enums.SlideHeight;
import org.sbs.bears.robotframework.controllers.SlideHeightController;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

public class AutonomousBrain {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideHeightController slideHCtrl;
    SlideExtensionController slideExtCtrl;
    IntakeController intakeCtrl;


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
        THREE_SLIDE_OUT_IN,
        FOUR_BACKWARD
    }

    double iniTime= 0;

    public AutonomousBrain(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode)
    {
        this.mode = mode;
        this.robot = new Robot(hardwareMap,telemetry,mode);
        this.CVctrl = robot.getCVctrl();
        this.RRctrl = robot.getRRctrl();
        this.slideHCtrl = robot.getSlideHCtrl();
        this.slideExtCtrl = robot.getSlideExtCtrl();
        this.intakeCtrl = robot.getIntakeCtrl();
    }
    public void launch()
    {
        iniTime = NanoClock.system().seconds();
    }
    public void doAutonAction()
    {
        switch(majorState)
        {
            case STOPPED:
                switch(mode) {
                    case BlueSimple:
                        RRctrl.setPos(startPositionBSimp);
                        break;
                    case BlueSpline:
                        RRctrl.setPos(startPositionBSpl);
                        break;
                    case RedSimple:
                        RRctrl.setPos(startPositionRSimp);
                        break;
                    case RedSpline:
                        RRctrl.setPos(startPositionRSpl);
                        break;

                }
                majorState = AutonomousStates.ONE_READ_DUCK;
                return;
            case ONE_READ_DUCK:
                heightFromDuck = CVctrl.getWhichTowerHeight();
                majorState = AutonomousStates.TWO_SET_SLIDE_HEIGHT;
                return;
            case TWO_SET_SLIDE_HEIGHT:
                switch(heightFromDuck)
                {
                    case ONE:
                        slideHCtrl.setSlideHeight(SlideHeight.ONE);
                        break;
                    case TWO:
                        slideHCtrl.setSlideHeight(SlideHeight.TWO);
                        break;
                    default:
                        slideHCtrl.setSlideHeight(SlideHeight.THREE_CLOSE);
                }
                majorState = AutonomousStates.THREE_DEPOSIT_BOX;
                return;
            case THREE_DEPOSIT_BOX:
                slideExtCtrl.extendDropRetract();
                if(mode.equals(AutonomousMode.RedSimple) || mode.equals(AutonomousMode.BlueSimple)) {
                    majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                }
                else {
                    majorState = AutonomousStates.FOUR_B_SET_SLIDE_HEIGHT_3;
                }
                return;
            case FOUR_B_SET_SLIDE_HEIGHT_3:
                slideHCtrl.setSlideHeight(SlideHeight.THREE_CLOSE);
                majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                return;
            case FOUR_DRIVE_TO_WAREHOUSE:
                switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    case BlueSpline:
                        RRctrl.followSplineTrajWarehouse(true);
                        RRctrl.followLineToSpline(wareHousePickupPositionBSpl);
                    case RedSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSimp);
                    case RedSpline:
                        RRctrl.followSplineTrajWarehouse(false);
                        RRctrl.followLineToSpline(wareHousePickupPositionRSpl);

                }
                majorState = AutonomousStates.FIVE_BACK_FORTH;
                return;
            case FIVE_BACK_FORTH:
                doBackForth();
                if(minorState == AutonomousBackForthSubStates.STOPPED)
                {
                    minorState = AutonomousBackForthSubStates.ONE_INTAKE;
                    return;
                }
                double currentTime = NanoClock.system().seconds();
                if(currentTime-iniTime > 25) {
                    majorState = AutonomousStates.SIX_PARKING_WAREHOUSE;
                }
                return;
            case SIX_PARKING_WAREHOUSE:
                switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    case BlueSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSpl);
                    case RedSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSimp);
                    case RedSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSpl);
                }
                majorState = AutonomousStates.FINISHED;
                return;
            case FINISHED:
                return;

        }
    }

    public void doBackForth()
    {
        switch(minorState)
        {
            case ONE_INTAKE:
                intakeCtrl.waitForIntake();
                minorState = AutonomousBackForthSubStates.TWO_FORWARD;
                return;
            case TWO_FORWARD:
                switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(depositObjectPositionBsimp);
                    case BlueSpline:
                        RRctrl.followLineToSpline(depositObjectPositionBspl);
                    case RedSimple:
                        RRctrl.followLineToSpline(depositObjectPositionRsimp);
                    case RedSpline:
                        RRctrl.followLineToSpline(depositObjectPositionRspl);
                }
                minorState = AutonomousBackForthSubStates.THREE_SLIDE_OUT_IN;
                return;
            case THREE_SLIDE_OUT_IN:
                switch(mode)
                {
                    case BlueSimple:
                    case RedSimple:
                        slideHCtrl.setSlideHeight(SlideHeight.THREE_CLOSE);
                        break;
                    case RedSpline:
                    case BlueSpline:
                        slideHCtrl.setSlideHeight(SlideHeight.THREE_FAR);
                }
                slideExtCtrl.extendDropRetract();
                minorState = AutonomousBackForthSubStates.FOUR_BACKWARD;
                return;
            case FOUR_BACKWARD:
                switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    case BlueSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSpl);
                    case RedSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSimp);
                    case RedSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSpl);
                }
                minorState = AutonomousBackForthSubStates.ONE_INTAKE;
                return;

        }
    }




    public static Pose2d startPositionBSimp = new Pose2d(6.5,65.5,0);
    public static Pose2d startPositionBSpl = new Pose2d(6.5,65.5,0);
    public static Pose2d startPositionRSimp = new Pose2d(6.5,-65.5,Math.PI);
    public static Pose2d startPositionRSpl = new Pose2d(6.5,-65.5,Math.PI);

    public static Pose2d wareHousePickupPositionBSimp = new Pose2d(54.5,65.5,0);
    public static Pose2d wareHousePickupPositionBSpl = new Pose2d(71, 34, -Math.PI/2);
    public static Pose2d wareHousePickupPositionRSimp = new Pose2d(54.5,-65.5,Math.PI);
    public static Pose2d wareHousePickupPositionRSpl = new Pose2d(71, -34, Math.PI/2);

    public static Pose2d depositObjectPositionBsimp = new Pose2d(-12.25,65.5,0);
    public static Pose2d depositObjectPositionBspl = new Pose2d(0,0,0);
    public static Pose2d depositObjectPositionRsimp = new Pose2d(-12.25,-65.5,Math.PI);
    public static Pose2d depositObjectPositionRspl = new Pose2d(0,0,0);



}
