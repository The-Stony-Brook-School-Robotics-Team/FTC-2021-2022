package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideExtensionController;
import org.sbs.bears.robotframework.enums.SlideHeight;
import org.sbs.bears.robotframework.controllers.SlideHeightController;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;
import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import java.util.concurrent.atomic.AtomicReference;


public class AutonomousBrain {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideHeightController slideHCtrl;
    SlideExtensionController slideExtCtrl;
    IntakeController intakeCtrl;

    Telemetry tel;
    HardwareMap hwMap;

    public  AutonomousStates majorState = AutonomousStates.STOPPED;
    public  AutonomousBackForthSubStates minorState = AutonomousBackForthSubStates.STOPPED;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;



    enum AutonomousStates {
        STOPPED,
        ONE_READ_DUCK,
        TWO_SET_SLIDE_HEIGHT,
        THREE_DEPOSIT_BOX,
        THREE_CAROUSEL,
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

    public AutonomousBrain(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) // call in init.
    {
        this.mode = mode;
        this.hwMap = hardwareMap;
        this.tel = telemetry;
        this.robot = new Robot(hardwareMap,telemetry,mode);
        this.CVctrl = robot.getCVctrl();
        this.RRctrl = robot.getRRctrl();
        this.slideHCtrl = robot.getSlideHCtrl();
        this.slideExtCtrl = robot.getSlideExtCtrl();
        this.intakeCtrl = robot.getIntakeCtrl();
    }
    public void launch() // call this method before loop, so start method.
    {
        iniTime = NanoClock.system().seconds();
    }
    public void doAutonAction() // call in loop (once per loop pls)
    {
        switch(majorState)
        {
            case STOPPED:
                doAnalysisMaster = true;
                switch(mode) {
                    case BlueSimple:
                        RRctrl.setPos(startPositionBSimp);
                        break;
                    case BlueSpline:
                        RRctrl.setPos(startPositionBSpl);
                        break;
                    case BlueFull:
                        RRctrl.setPos(startPositionBFull);
                        break;
                    case RedSimple:
                        RRctrl.setPos(startPositionRSimp);
                        break;
                    case RedSpline:
                        RRctrl.setPos(startPositionRSpl);
                        break;
                    case RedFull:
                        RRctrl.setPos(startPositionRFull);
                        break;

                }
                majorState = AutonomousStates.ONE_READ_DUCK;
                return;
            case ONE_READ_DUCK:
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ",heightFromDuck.toString());
                tel.addData("height: ",heightFromDuck);
                tel.update();
                majorState = AutonomousStates.TWO_SET_SLIDE_HEIGHT;
                CVctrl.shutDown();
                return;
            case TWO_SET_SLIDE_HEIGHT:

                switch(heightFromDuck)
                {
                    case ONE:
                        slideHCtrl.setSlideHeight(SlideHeight.ONE_CAROUSEL);
                        break;
                    case TWO:
                        slideHCtrl.setSlideHeight(SlideHeight.TWO_CAROUSEL);
                        break;
                    default:
                        slideHCtrl.setSlideHeight(SlideHeight.THREE_CAROUSEL);
                }

                majorState = AutonomousStates.THREE_CAROUSEL;
                return;
            /*case THREE_DEPOSIT_BOX:
                slideExtCtrl.extendDropRetract();
                if(mode.equals(AutonomousMode.RedSimple) || mode.equals(AutonomousMode.BlueSimple)) {
                    slideHCtrl.setSlideHeight(SlideHeight.THREE_CLOSE);
                    majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                }
                else {
                    majorState = AutonomousStates.FOUR_B_SET_SLIDE_HEIGHT_3;
                }
                if(mode.equals(AutonomousMode.BlueFull) || mode.equals(AutonomousMode.RedFull)) {
                    majorState = AutonomousStates.THREE_OPT_CAROUSEL;
                }
                return;*/
            case THREE_CAROUSEL:
                if(mode.equals(AutonomousMode.BlueFull)) {
                    RRctrl.followLineToSpline(duckSpinningPositionB);
                    new Thread(()->{slideExtCtrl.extendDropRetract();}).start();
                    spinDuck(true);

                    mode = AutonomousMode.BlueSimple;
                }
                if(mode.equals(AutonomousMode.RedFull)) {
                    RRctrl.followLineToSpline(duckSpinningPositionR);
                    new Thread(()->{slideExtCtrl.extendDropRetract();}).start();
                    spinDuck(true);
                    slideExtCtrl.extendDropRetract();
                    mode = AutonomousMode.RedSimple;
                }

                majorState = AutonomousStates.FOUR_B_SET_SLIDE_HEIGHT_3;
                return;
            case FOUR_B_SET_SLIDE_HEIGHT_3:
                slideHCtrl.setSlideHeight(SlideHeight.THREE_CLOSE);
                majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                //majorState = AutonomousStates.FINISHED;
                return;
            case FOUR_DRIVE_TO_WAREHOUSE:
                switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimpIntermediate);
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    /*case BlueSpline:
                        RRctrl.followSplineTrajWarehouse(true);
                        RRctrl.followLineToSpline(wareHousePickupPositionBSpl);
                    case RedSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSimp);
                    case RedSpline:
                        RRctrl.followSplineTrajWarehouse(false);
                        RRctrl.followLineToSpline(wareHousePickupPositionRSpl);
*/
                }
                //majorState = AutonomousStates.FINISHED;
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
                Object externMutex = new Object();
                AtomicReference<Boolean> stopSignal = new AtomicReference<>(Boolean.getBoolean("false"));
                RRctrl.doForwardHaltableTrajectory(20,4,50,40, stopSignal.get(),externMutex);
                new Thread(()->{
                    intakeCtrl.waitForIntake();
                    synchronized (externMutex) {
                        stopSignal.set(Boolean.getBoolean("true"));} // will halt trajectory in separate thread
                }).start();
                RRctrl.stopRobot(); // make sure robot is stopped.
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
                switch(mode) // should have already set slide height before.
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


    public void spinDuck(boolean qBlue)
    {
        try {
            DcMotor duckSpinner = hwMap.get(DcMotor.class, "duck");
            duckSpinner.setPower(qBlue ? -.3 : .3);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            duckSpinner.setPower(0);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        catch(Exception e) {
            Log.d("AutonomousBrain","DuckSpinner Failed");
        }
    }



    public static Pose2d startPositionBSimp = new Pose2d(6.5,65.5,0);
    public static Pose2d startPositionBSpl = new Pose2d(6.5,65.5,0);
    public static Pose2d startPositionBFull = new Pose2d(-42,66,0);
    public static Pose2d startPositionRSimp = new Pose2d(6.5,-65.5,Math.PI);
    public static Pose2d startPositionRSpl = new Pose2d(6.5,-65.5,Math.PI);
    public static Pose2d startPositionRFull = new Pose2d(-42,-66,Math.PI);

    public static Pose2d duckSpinningPositionB = new Pose2d(-60, 63, Math.toRadians(48));
    public static Pose2d duckSpinningPositionR = new Pose2d(-60, -63, Math.toRadians(-48));

    public static Pose2d wareHousePickupPositionBSimpIntermediate = new Pose2d(-45,66,0);
    public static Pose2d wareHousePickupPositionRSimpIntermediate = new Pose2d(-45,-66,0);

    public static Pose2d wareHousePickupPositionBSimp = new Pose2d(35,65.5,0);



    public static Pose2d wareHousePickupPositionBSpl = new Pose2d(71, 34, -Math.PI/2);
    public static Pose2d wareHousePickupPositionRSimp = new Pose2d(54.5,-65.5,Math.PI);
    public static Pose2d wareHousePickupPositionRSpl = new Pose2d(71, -34, Math.PI/2);

    public static Pose2d depositObjectPositionBsimp = new Pose2d(-12.25,65.5,0);
    public static Pose2d depositObjectPositionBspl = new Pose2d(71,19,-Math.PI/2);
    public static Pose2d depositObjectPositionRsimp = new Pose2d(-12.25,-65.5,Math.PI);
    public static Pose2d depositObjectPositionRspl = new Pose2d(71,-10,Math.PI/2);



}
