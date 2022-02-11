package org.firstinspires.ftc.teamcode.common.autonomous;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.sharedResources.SharedData;
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

@Config
public class AutonomousBrainMerged {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideController slideCtrl;
    IntakeControllerBlue intakeCtrlBlue;
    IntakeControllerRed intakeCtrlRed;
    DuckCarouselController duckCtrl;
    Pose2d iniDropPosition = depositPositionAllianceBlueTOP;

    RevBlinkinLedDriver leds;
    NormalizedColorSensor normalizedColorSensor;
    RevColorSensorV3 colorNew;

    Telemetry tel;
    HardwareMap hwMap;
    boolean qObjectInRobot = false;

    public MajorAutonomousState majorState = MajorAutonomousState.STOPPED;
    public MinorAutonomousState minorState = MinorAutonomousState.STOPPED;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;

    SlideTarget iniTarget; // decides randomized position
    SlideTarget normalTarget = SlideTarget.TOP_DEPOSIT;

    enum MajorAutonomousState {
        STOPPED,
        ONE_CAMERA_READ,
        TWO_DEPOSIT_INI_BLOCK,
        THREE_FIRST_INTAKE,
        FOUR_BACK_FORTH,
        FIVE_PARKING_CLEANUP,
        FINISHED
    }
    enum MinorAutonomousState {
        STOPPED,
        ONE_INTAKE,
        TWO_PREP_DEPOSIT,
        THREE_DEPOSIT,
    }

    double iniTemps = 0;

    public AutonomousBrainMerged(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) // call in init.
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
        this.leds = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");

        RRctrl.setPos(startPositionBlue);
        intakeCtrlBlue.setState(IntakeState.PARK);
        intakeCtrlRed.setState(IntakeState.PARK); // to prevent from moving around
        //normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        colorNew = hardwareMap.get(RevColorSensorV3.class, "color");
        colorNew.write8(BroadcomColorSensor.Register.LS_MEAS_RATE,0x01010000); // see pdf page 20 // increase speed
        colorNew.write8(BroadcomColorSensor.Register.PS_MEAS_RATE,0x00000001); // see pdf page 19 // increase speed
        slideCtrl.dumperServo.setPosition(slideCtrl.dumperPosition_CLOSED); // init only
    }
    public void start() // call this method before loop, so start method.
    {
        iniTemps = NanoClock.system().seconds();
    }
    public void doStateAction() // call in loop (once per loop pls)
    {
        switch(majorState) {
            case STOPPED:
                doAnalysisMaster = true;
                majorState = MajorAutonomousState.ONE_CAMERA_READ;
                return;
            case ONE_CAMERA_READ:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ", heightFromDuck.toString());
                CVctrl.shutDown();
                switch (heightFromDuck) {
                    case ONE:
                        iniTarget = SlideTarget.BOTTOM_DEPOSIT;
                        iniDropPosition = depositPositionAllianceBlueBOT;
                        break;
                    case TWO:
                        iniTarget = SlideTarget.MID_DEPOSIT;
                        iniDropPosition = depositPositionAllianceBlueMID;
                        break;
                    case THREE:
                        iniTarget = SlideTarget.TOP_DEPOSIT;
                        iniDropPosition = depositPositionAllianceBlueTOP;
                        break;
                }
                majorState = MajorAutonomousState.TWO_DEPOSIT_INI_BLOCK;
                return;
            case TWO_DEPOSIT_INI_BLOCK:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                RRctrl.followLineToSpline(iniDropPosition);
                slideCtrl.extendDropRetract(iniTarget);
                Log.d("AutonBrain","Slide drop complete");



                qObjectInRobot = false; // reset
                majorState = MajorAutonomousState.THREE_FIRST_INTAKE;
                return;
            case THREE_FIRST_INTAKE:
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                Log.d("AutonBrain","reset status and init for intake");
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue2);
                Log.d("AutonBrain","Threads Init");
                prepareIntakeThreads();
                ///RRctrl.followLineToSpline(warehousePickupPositionBlue);
                Log.d("AutonBrain","Traj Init");
                RRctrl.autonomousDoFirstIntakeTraj();
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                Log.d("AutonBrain","Traj Ended");
                RRctrl.stopRobot();
                doPostIntakeDecisionMaking();
                majorState = MajorAutonomousState.FOUR_BACK_FORTH;
                return;
            case FOUR_BACK_FORTH:
                if(minorState == MinorAutonomousState.ONE_INTAKE)
                {
                    minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
                    return;
                }
                doGoBack();
                // time check
               double currentTime = NanoClock.system().seconds();
                if(currentTime- iniTemps > 25) {
                    Log.d("AutonBrain","Time Constraint: parking");
                    majorState = MajorAutonomousState.FIVE_PARKING_CLEANUP;
                }
                return;
            case FIVE_PARKING_CLEANUP:
                Log.d("AutonBrain","parking1");
                intakeCtrlBlue.setState(IntakeState.PARK);
                /*
                Log.d("AutonBrain","parking2");
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                Log.d("AutonBrain","parking3");
                RRctrl.followLineToSpline(parkingPositionBlue);
                Log.d("AutonBrain","parking4");
                */
                //if(!RRctrl.isInWarehouse()) {RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);}
                RRctrl.followLineToSpline(parkingPositionBlue);
                SharedData.autonomousLastPosition = RRctrl.getPos();
                majorState = MajorAutonomousState.FINISHED;
                return;
            case FINISHED:
                return;

        }
    }
    private void doPostIntakeDecisionMaking() {
        if(majorState != MajorAutonomousState.FOUR_BACK_FORTH) {majorState = MajorAutonomousState.FOUR_BACK_FORTH;}
        if(qObjectInRobot)
        {
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
            Log.d("AutonBrain","Continuing to deposit");
            return;
        }
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        RRctrl.followLineToSpline(warehousePickupPositionBlue);
        Log.d("AutonBrain","Retrying to find a block");
        minorState = MinorAutonomousState.ONE_INTAKE;
    }
    private void prepareIntakeThreads() {
        Log.d("AutonBrain","Creating Intake Helper Threads");
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        Log.d("AutonBrain","Current Status: init first intake: itemBool: " + qObjectInRobot + " intakeStatus " + intakeCtrlBlue.isObjectInPayload());
        new Thread(()->{ // debugging
            boolean isInState = minorState.equals(AutonomousBrainMerged.MinorAutonomousState.ONE_INTAKE);
            while(isInState && !qObjectInRobot)
            {
                isInState = minorState.equals(AutonomousBrainMerged.MinorAutonomousState.ONE_INTAKE);
                if(AutonomousBlueFull.gamepad.b) // manual override
                {
                    qObjectInRobot =true;
                    Sleep.sleep(100);
                    break;
                }
            }
            if(qObjectInRobot) {
                RRctrl.haltTrajectory();
                minorState = AutonomousBrainMerged.MinorAutonomousState.TWO_PREP_DEPOSIT;
                intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly();
            }
        }).start();
        new Thread(()->{
            boolean isInState = minorState.equals(MinorAutonomousState.ONE_INTAKE);
            Log.d("AutonBrainThread","Status0: scoop: " + qObjectInRobot +" state " + isInState);
            while(!qObjectInRobot && isInState)
            {
                Sleep.sleep(10);
                isInState = minorState.equals(MinorAutonomousState.ONE_INTAKE);
                qObjectInRobot = intakeCtrlBlue.isObjectInPayload();
                //Log.d("AutonBrainThread","Status: scoop: " + qObjectInRobot +" state " + isInState);
            }
            Log.d("AutonBrainThread","Status2: scoop: " + qObjectInRobot +" state " + isInState);
            if(qObjectInRobot)
            {
                RRctrl.stopTrajectory();
                intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly();
                Log.d("AutonBrainThread","Status: loaded");
            }
        }).start();
    }

    public void doGoBack()
    {
        switch(minorState)
        {
            case STOPPED:
                // No associated action
                return;
            case ONE_INTAKE:

                if(qObjectInRobot || intakeCtrlBlue.isObjectInPayload())
                {
                    //We have a block
                    Log.d("AutonBrain","Missed block on last run, proceeding.");
                    minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
                    return;
                }
                prepareIntakeThreads();
                intakeCtrlBlue.setState(IntakeState.BASE);
                if(!RRctrl.isInWarehouse()) { // outside => need to come back from deposit.
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                    qObjectInRobot = false; // reset
                    Log.d("AutonBrain","intake prepped");
                    Log.d("AutonBrain","reset status and init for intake");
                    RRctrl.autonomousPrepAndIntakeFromDeposit();
                }
                else {
                    Log.d("AutonBrain", "Forward init");
                    RRctrl.forward(distanceIntake, velocityIntake);
                }
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                Log.d("AutonBrain","Halted or Done");
                RRctrl.stopRobot();
                // stopped
                doPostIntakeDecisionMaking();
                return;

            case TWO_PREP_DEPOSIT:

               /* new Thread(()->{
                    // do white line repositioning
                    boolean isInState = minorState.equals(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    while(isInState)
                    {
                        isInState = minorState.equals(MinorAutonomousState.TWO_PREP_DEPOSIT);
                        if(colorNew.getNormalizedColors().alpha > Configuration.colorSensorWhiteAlpha)
                        {
                            Log.d("AutonBrainThread","White tape pos x: " + RRctrl.getPos().getX() + " actual " + whiteLinePos.getX());
                            //RRctrl.setPos(whiteLinePos);
                            break; // done.
                        }
                    }
                }).start();
                */
                Log.d("AutonBrain","Prepare for drop off");
                RRctrl.doBlueDepositTrajectoryNoTurn();
                Log.d("AutonBrain","Preparation Complete");
                minorState = MinorAutonomousState.THREE_DEPOSIT;
                return;
            case THREE_DEPOSIT:
                slideCtrl.extendDropRetract(normalTarget);
                Log.d("AutonBrain","Slide drop complete");
                minorState = MinorAutonomousState.ONE_INTAKE; // go back to step one
                return;

        }
    }

    public static Pose2d startPositionBlue = new Pose2d(14,65.5,0);
    public static Pose2d warehousePickupPositionBlue = new Pose2d(35,70,0);
    public static Pose2d depositPrepPositionBlue = new Pose2d(30,70,0);
    public static Pose2d depositPositionBlueNoTurn = new Pose2d(-17,70,0);
    public static Pose2d depositPositionAllianceBlueTOP = new Pose2d(5.58,64.47,-Math.toRadians(55));
    public static Pose2d depositPositionAllianceBlueMID = new Pose2d(5.58,64.47,-Math.toRadians(56));
    public static Pose2d depositPositionAllianceBlueBOT = new Pose2d(5.58,64.47,-Math.toRadians(59));
    public static Pose2d depositPositionAllianceBlue2 = new Pose2d(5.58,64.47,-Math.toRadians(48));
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14,75,0);
    public static Pose2d resetPositionB4WarehouseBlue2 = new Pose2d(14,70,0);
    public static Pose2d parkingPositionBlue = new Pose2d(50,70,0);
    public static Pose2d whiteLinePos = new Pose2d(29.5,65.5,0);
    public static double velocityIntake = 15;
    public static double distanceIntake = 40;


}
