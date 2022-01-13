package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeSide;
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
    IntakeController intakeCtrl;
    IntakeController intakeCtrl2;
    DuckCarouselController duckCtrl;

    Telemetry tel;
    HardwareMap hwMap;
     int counter = 0;
     int counterDidntFind = 0;
     boolean didIScoopAnItem = false;

    public  AutonomousStates majorState = AutonomousStates.STOPPED;
    public  AutonomousBackForthSubStates minorState = AutonomousBackForthSubStates.STOPPED;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;


    SlideTarget targetCarousel;
    SlideTarget targetNormal = SlideTarget.THREE_DEPOSIT;

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
        this.intakeCtrl = robot.getIntakeCtrl();
        this.intakeCtrl2 = new IntakeController(hardwareMap,telemetry, IntakeSide.RED);
        this.duckCtrl = robot.getDuckCtrl();
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
                intakeCtrl.setState(IntakeState.PARK);
                intakeCtrl2.setState(IntakeState.PARK); // to prevent from moving around
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
                        targetCarousel = SlideTarget.ONE_CAROUSEL;
                        break;
                    case TWO:
                        targetCarousel = SlideTarget.TWO_CAROUSEL;
                        break;
                    default:
                        targetCarousel = SlideTarget.THREE_CAROUSEL;
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
                    new Thread(()->{slideCtrl.extendDropRetract(targetCarousel);}).start();
                    spinDuck(true);

                    mode = AutonomousMode.BlueSimple;
                }
                if(mode.equals(AutonomousMode.RedFull)) {
                    RRctrl.followLineToSpline(duckSpinningPositionR);
                    new Thread(()->{slideCtrl.extendDropRetract(targetCarousel);}).start();
                    spinDuck(false);
                    mode = AutonomousMode.RedSimple;
                }

                majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                return;
            /*case FOUR_B_SET_SLIDE_HEIGHT_3:
                majorState = AutonomousStates.FOUR_DRIVE_TO_WAREHOUSE;
                return;*/
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
                intakeCtrl.setState(IntakeState.PARK);
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
                Object externMutex = new Object();
                AtomicReference<Boolean> stopSignal = new AtomicReference<>(Boolean.getBoolean("false"));
                intakeCtrl.setState(IntakeState.BASE);
                new Thread(()->{
                    boolean isInState = true;
                    synchronized (externMutex) {
                        isInState = minorState.equals(AutonomousBackForthSubStates.ONE_INTAKE);
                    }
                    while(!intakeCtrl.isObjectInPayload() && isInState) {
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
                        didIScoopAnItem = intakeCtrl.isObjectInPayload();
                        Log.d("AutonBrain","didIScoopAnItem: " + didIScoopAnItem);
                        if(didIScoopAnItem) {
                            stopSignal.set(Boolean.getBoolean("true"));
                            RRctrl.haltTrajectory();
                            RRctrl.stopRobot();
                        }
                    } // will halt trajectory in separate thread
                    intakeCtrl.loadItemIntoSlideForAutonomousOnly(); // approved usage
                }).start();
                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                RRctrl.forward(20,10);
                if(!didIScoopAnItem) {
                    return; // try again
                }


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
                }*/
                RRctrl.stopRobot(); // make sure robot is stopped.

                minorState = AutonomousBackForthSubStates.TWO_DEPOSIT;
                return;
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
                minorState = AutonomousBackForthSubStates.THREE_SLIDE_OUT_IN;
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


    public void spinDuck(boolean qBlue)
    {

        if(qBlue)
        {
            duckCtrl.spinOneDuck();
        }
        else {
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
    public static Pose2d wareHousePickupPositionBSimp = new Pose2d(30,65.5,0);
    public static Pose2d wareHousePickupPositionBSimp2 = new Pose2d(38,65.5,0);


    public static Pose2d wareHousePickupPositionBSpl = new Pose2d(71, 34, -Math.PI/2);
    public static Pose2d wareHousePickupPositionRSimp = new Pose2d(54.5,-65.5,Math.PI);
    public static Pose2d wareHousePickupPositionRSpl = new Pose2d(71, -34, Math.PI/2);

    public static Pose2d depositObjectPositionBsimp = new Pose2d(-19,65.5,0);




    public static Pose2d depositObjectPositionBspl = new Pose2d(71,19,-Math.PI/2);
    public static Pose2d depositObjectPositionRsimp = new Pose2d(-12.25,-65.5,Math.PI);
    public static Pose2d depositObjectPositionRspl = new Pose2d(71,-10,Math.PI/2);



}
