package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;
import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import java.util.concurrent.atomic.AtomicReference;


public class AutonomousBrain {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideController slideCtrl;
    IntakeControllerBlue intakeCtrlBlue;
    IntakeControllerRed intakeCtrlRed;
    DuckCarouselController duckCtrl;

    Telemetry tel;
    HardwareMap hwMap;
     int counter = 0;
     int counterDidntFind = 0;
     boolean didIScoopAnItem = false;

    public  AutonomousStates majorState = AutonomousStates.STOPPED;
    public  AutonomousBackForthSubStates minorState = AutonomousBackForthSubStates.STOPPED;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;

    Pose2d carouselDropPosition = duckSpinningPositionB3;
    Pose2d carouselDropPositionFlush = duckSpinningPositionBflush3;

    SlideTarget targetCarousel;
    SlideTarget targetNormal = SlideTarget.THREE_DEPOSIT;

    enum AutonomousStates {
        STOPPED,
        ONE_READ_DUCK,
        TWO_SET_SLIDE_HEIGHT,
        THREE_DEPOSIT_BOX,
        TWO_CAROUSEL,
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
        TWO_DEPOSIT,
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
        this.slideCtrl = robot.getSlideCtrl();
        this.intakeCtrlBlue = robot.getIntakeCtrlBlue();
        this.intakeCtrlRed = robot.getIntakeCtrlRed();
        this.duckCtrl = robot.getDuckCtrl();
        intakeCtrlBlue.setState(IntakeState.PARK);
        intakeCtrlRed.setState(IntakeState.PARK); // to prevent from moving around
        RRctrl.setPos(startPositionBFull);
    }
    public void launch() // call this method before loop, so start method.
    {
        iniTime = NanoClock.system().seconds();
    }
    public void doAutonAction() // call in loop (once per loop pls)
    {
        switch(majorState) {
            case STOPPED:
                doAnalysisMaster = true;
                majorState = AutonomousStates.ONE_READ_DUCK;
                return;
            case ONE_READ_DUCK:
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ", heightFromDuck.toString());
                CVctrl.shutDown();
                switch (heightFromDuck) {
                    case ONE:
                        targetCarousel = SlideTarget.ONE_CAROUSEL;
                        carouselDropPosition = duckSpinningPositionB1;
                        carouselDropPositionFlush = duckSpinningPositionBflush1;
                        break;
                    case TWO:
                        targetCarousel = SlideTarget.TWO_CAROUSEL;
                        carouselDropPosition = duckSpinningPositionB2;
                        carouselDropPositionFlush = duckSpinningPositionBflush2;
                        break;
                    case THREE:
                        targetCarousel = SlideTarget.THREE_CAROUSEL;
                        carouselDropPosition = duckSpinningPositionB3;
                        carouselDropPositionFlush = duckSpinningPositionBflush3;
                }
                majorState = AutonomousStates.TWO_CAROUSEL;
                return;
            case TWO_CAROUSEL:
                RRctrl.followLineToSpline(carouselDropPosition);
                RRctrl.followLineToSplineAsync(carouselDropPositionFlush);
                new Thread(()->{
                    slideCtrl.targetParams = targetCarousel;
                    slideCtrl.extendDropRetract(targetCarousel);
                    Log.d("AutonBrain","slide finished");
                }).start();
                duckCtrl.spinOneDuck();
                Log.d("AutonBrain","duck finished");
                RRctrl.stopTrajectory();
                majorState = AutonomousStates.FINISHED;

                mode = AutonomousMode.BlueSimple;
                majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                return;

            case FOUR_DRIVE_TO_WAREHOUSE:
                RRctrl.followLineToSpline(wareHousePickupPositionBSimpIntermediate);
                RRctrl.followLineToSpline(wareHousePickupPositionBSimpIntermediate2);
                RRctrl.setPos(new Pose2d(-35,65.5,0));
                RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                majorState = AutonomousStates.FIVE_BACK_FORTH;
                return;
            case FIVE_BACK_FORTH:
                doBackForth();
                if(minorState == AutonomousBackForthSubStates.STOPPED)
                {
                    minorState = AutonomousBackForthSubStates.ONE_INTAKE;
                    return;
                }
                // time check
               double currentTime = NanoClock.system().seconds();
                if(currentTime-iniTime > 25) {
                    majorState = AutonomousStates.SIX_PARKING_WAREHOUSE;
                }
                return;
            case SIX_PARKING_WAREHOUSE:
                /*switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    case BlueSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSpl);
                    case RedSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSimp);
                    case RedSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSpl);
                }*/
                intakeCtrlBlue.setState(IntakeState.PARK);
                RRctrl.followLineToSpline(wareHousePickupPositionBSimp2);
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
            case STOPPED:
                // do nothing
                return;
            case ONE_INTAKE:
                new Thread(()->{
                    boolean isInState = minorState.equals(AutonomousBackForthSubStates.ONE_INTAKE);
                    while(!didIScoopAnItem && isInState)
                    {
                        Sleep.sleep(10);
                        isInState = minorState.equals(AutonomousBackForthSubStates.ONE_INTAKE);
                        didIScoopAnItem = intakeCtrlBlue.isObjectInPayload();
                    }
                    if(didIScoopAnItem)
                    {
                        RRctrl.stopTrajectory();
                        RRctrl.stopRobot();
                    }
                }).start();
                RRctrl.forward(20,10);
                // stopped
                if(didIScoopAnItem)
                {
                    minorState = AutonomousBackForthSubStates.TWO_DEPOSIT;
                    return;
                }
                RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                return;
            /*case ONE_INTAKE:
                Object externMutex = new Object();
                AtomicReference<Boolean> stopSignal = new AtomicReference<>(Boolean.getBoolean("false"));
                intakeCtrlBlue.setState(IntakeState.BASE);

                new Thread(()->{
                    boolean isInState = true;
                    synchronized (externMutex) {
                        isInState = minorState.equals(AutonomousBackForthSubStates.ONE_INTAKE);
                    }
                    while(!intakeCtrlBlue.isObjectInPayload() && isInState) {
                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        synchronized (externMutex) {
                            isInState = minorState.equals(AutonomousBackForthSubStates.ONE_INTAKE);
                        }
                    }
                    synchronized (externMutex) {
                        didIScoopAnItem = intakeCtrlBlue.isObjectInPayload();
                        Log.d("AutonBrain","didIScoopAnItem: " + didIScoopAnItem + " " + isInState);
                        if(didIScoopAnItem) {
                            stopSignal.set(Boolean.getBoolean("true"));
                            RRctrl.haltTrajectory();
                            RRctrl.stopRobot();
                        }
                    } // will halt trajectory in separate thread
                    intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly(); // approved usage
                }).start();
                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                if(didIScoopAnItem)
                {
                    // continue
                    Log.d("AutonBrain","continuing to deposit");
                    minorState = AutonomousBackForthSubStates.TWO_DEPOSIT;
                    return;
                }
                Log.d("AutonBrain","starting forward: " + didIScoopAnItem + " ");
                RRctrl.forward(20,10); // interruptible
                Log.d("AutonBrain","done forward: " + didIScoopAnItem + " ");
                if(!didIScoopAnItem) {
                    RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    Log.d("AutonBrain","trying again");
                    return; // try again
                }
                Log.d("AutonBrain","continuing to deposit");



                //RRctrl.doForwardHaltableTrajectory(30,3,50,40, stopSignal.get(),externMutex);
                /*if(!didIScoopAnItem) {
                    RRctrl.followLineToSpline(wareHousePickupPositionBSimp2);
                    RRctrl.strafeR(5);
                    counterDidntFind++;
                    Log.d("AutonBrain","Didnt find item, " + counterDidntFind + "th time");
                    return;
                }
                synchronized (externMutex) {
                    didIScoopAnItem = false;
                }
                RRctrl.stopRobot(); // make sure robot is stopped.
                minorState = AutonomousBackForthSubStates.TWO_DEPOSIT;
                return;*/
            case TWO_DEPOSIT: // TODO implement go forward and then turn
                /*switch(mode) {
                    case BlueSimple:
                    case BlueFull:
                        RRctrl.followLineToSpline(depositObjectPositionBsimp);
                    case BlueSpline:
                        RRctrl.followLineToSpline(depositObjectPositionBspl);
                    case RedSimple:
                        RRctrl.followLineToSpline(depositObjectPositionRsimp);
                    case RedSpline:
                        RRctrl.followLineToSpline(depositObjectPositionRspl);
                }*/
                RRctrl.followLineToSpline(depositObjectPositionBsimp); // TODO reput switch for all possible autonomi
                //minorState = AutonomousBackForthSubStates.THREE_SLIDE_OUT_IN;
                minorState = AutonomousBackForthSubStates.STOPPED;
                majorState = AutonomousStates.FINISHED;
                counter++; // 1 after first run, 2 after second.
                /*while(!AutonomousBlueFull.gamepad.b) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }*/
                /*if (counter == 2) {
                    minorState = AutonomousBackForthSubStates.STOPPED;
                    majorState = AutonomousStates.STOPPED;
                    return;
                }*/
                //majorState = AutonomousStates.FINISHED; // stop.
                return;
            case THREE_SLIDE_OUT_IN:
                slideCtrl.extendDropRetract(targetNormal);
                minorState = AutonomousBackForthSubStates.FOUR_BACKWARD;
                return;
            case FOUR_BACKWARD:
                /*switch(mode) {
                    case BlueSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                    case BlueSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionBSpl);
                    case RedSimple:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSimp);
                    case RedSpline:
                        RRctrl.followLineToSpline(wareHousePickupPositionRSpl);
                }*/
                RRctrl.followLineToSpline(wareHousePickupPositionBSimp);
                minorState = AutonomousBackForthSubStates.ONE_INTAKE;
                return;

        }
    }

    public static Pose2d startPositionBSimp = new Pose2d(6.5,65.5,0);
    public static Pose2d startPositionBSpl = new Pose2d(6.5,65.5,0);
    public static Pose2d startPositionBFull = new Pose2d(-42,66,0);
    public static Pose2d startPositionRSimp = new Pose2d(6.5,-65.5,Math.PI);
    public static Pose2d startPositionRSpl = new Pose2d(6.5,-65.5,Math.PI);
    public static Pose2d startPositionRFull = new Pose2d(-42,-66,Math.PI);

    public static Pose2d duckSpinningPositionB3 = new Pose2d(-61, 63, Math.toRadians(50));
    public static Pose2d duckSpinningPositionB2 = new Pose2d(-64, 60, Math.toRadians(49));
    public static Pose2d duckSpinningPositionB1 = new Pose2d(-63, 61, Math.toRadians(47));
    public static Pose2d duckSpinningPositionBflush1 = new Pose2d(-64, 66, Math.toRadians(50));
    public static Pose2d duckSpinningPositionBflush2 = new Pose2d(-64, 66, Math.toRadians(49));
    public static Pose2d duckSpinningPositionBflush3 = new Pose2d(-64, 66, Math.toRadians(47));
    public static Pose2d duckSpinningPositionR = new Pose2d(-60, -63, Math.toRadians(-48));

    public static Pose2d wareHousePickupPositionBSimpIntermediate = new Pose2d(-45,63,0);
    public static Pose2d wareHousePickupPositionBSimpIntermediate2 = new Pose2d(-35,70,0);
    public static Pose2d wareHousePickupPositionRSimpIntermediate = new Pose2d(-45,-66,0);
    public static Pose2d wareHousePickupPositionBSimp = new Pose2d(40,65.5,0);
    public static Pose2d wareHousePickupPositionBSimp2 = new Pose2d(38,65.5,0);


    public static Pose2d wareHousePickupPositionBSpl = new Pose2d(71, 34, -Math.PI/2);
    public static Pose2d wareHousePickupPositionRSimp = new Pose2d(54.5,-65.5,Math.PI);
    public static Pose2d wareHousePickupPositionRSpl = new Pose2d(71, -34, Math.PI/2);

    public static Pose2d depositObjectPositionBsimp = new Pose2d(-19,65.5,0);
    // TODO TODO TODO MEASURE THE POSITION TO TURN AND DEPOSIT!!!!!
    // just replace the variable above.



    public static Pose2d depositObjectPositionBspl = new Pose2d(71,19,-Math.PI/2);
    public static Pose2d depositObjectPositionRsimp = new Pose2d(-12.25,-65.5,Math.PI);
    public static Pose2d depositObjectPositionRspl = new Pose2d(71,-10,Math.PI/2);



}
