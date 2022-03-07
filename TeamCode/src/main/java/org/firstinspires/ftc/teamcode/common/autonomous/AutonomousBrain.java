package org.firstinspires.ftc.teamcode.common.autonomous;

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.sharedResources.SharedData;
import org.sbs.bears.robotframework.Robot;
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

@Config
public class AutonomousBrain {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideController slideCtrl;
    IntakeControllerBlue intakeCtrlBlue;
    IntakeControllerRed intakeCtrlRed;
    DuckCarouselController duckCtrl;
    Pose2d iniDropPosition = depositPositionAllianceBlueTOP;

    int numberOfTrials = 1;
    RevBlinkinLedDriver leds;
    NormalizedColorSensor normalizedColorSensor;
    RevColorSensorV3 colorNew;

    Telemetry tel;
    HardwareMap hwMap;
    AtomicReference<Boolean> qObjectInRobot = new AtomicReference<>();
    AtomicReference<Boolean> qObjectIsLoaded = new AtomicReference<>();

    public AtomicReference<MajorAutonomousState> majorState = new AtomicReference<>();
    public AtomicReference<MinorAutonomousState> minorState = new AtomicReference<>();
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;

    SlideTarget iniTarget; // decides randomized position
    SlideTarget normalTarget = SlideTarget.TOP_DEPOSIT_AUTON;

    enum MajorAutonomousState {
        STOPPED,
        ONE_CAMERA_READ,
        TWO_DEPOSIT_INI_BLOCK,
        THREE_BACK_FORTH,
        FOUR_PARKING_CLEANUP,
        FINISHED
    }
    enum MinorAutonomousState {
        STOPPED,
        ONE_INTAKE,
        TWO_PREP_DEPOSIT,
        THREE_DEPOSIT,
        FOUR_RETURN_TO_INTAKE,
        FINISHED
    }

    double iniTemps = 0;
    boolean isBlue = true;

    public AutonomousBrain(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) // call in init.
    {
        majorState.set(MajorAutonomousState.STOPPED);
        minorState.set(MinorAutonomousState.STOPPED);
        qObjectInRobot.set(false);
        qObjectIsLoaded.set(false);
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
        isBlue = (mode == AutonomousMode.BlueStatesWarehouse);

        if (isBlue) {
            RRctrl.setPos(startPositionBlue);
        } else {
            RRctrl.setPos(startPositionRed);
        }
        intakeCtrlBlue.setState(IntakeState.DUMP);
        intakeCtrlRed.setState(IntakeState.DUMP); // to prevent from moving around
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
        switch(majorState.get()) {
            case STOPPED:
                doAnalysisMaster = true;
                majorState.set(MajorAutonomousState.ONE_CAMERA_READ);
                return;
            case ONE_CAMERA_READ:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ", heightFromDuck.toString());
                CVctrl.shutDown();
                switch (heightFromDuck) {
                    case ONE:
                        iniTarget = SlideTarget.BOTTOM_DEPOSIT;
                        iniDropPosition = isBlue ? depositPositionAllianceBlueBOT : depositPositionAllianceRedBOT;
                        break;
                    case TWO:
                        iniTarget = SlideTarget.MID_DEPOSIT;
                        iniDropPosition = isBlue ? depositPositionAllianceBlueMID : depositPositionAllianceRedMID;
                        break;
                    case THREE:
                        iniTarget = SlideTarget.TOP_DEPOSIT_AUTON;
                        iniDropPosition = isBlue ? depositPositionAllianceBlueTOP : depositPositionAllianceRedTOP;
                        break;
                }
                majorState.set(MajorAutonomousState.TWO_DEPOSIT_INI_BLOCK);
                return;
            case TWO_DEPOSIT_INI_BLOCK:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                RRctrl.followLineToSpline(iniDropPosition);
                slideCtrl.extendDropRetract(iniTarget);
                Log.d("AutonBrain","Slide drop complete");
                if(isBlue) {
                    RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                    intakeCtrlBlue.setState(IntakeState.BASE);
                    RRctrl.followLineToSpline(resetPositionB4WarehouseBlue2);
                    RRctrl.followLineToSpline(warehousePickupPositionBlue);
                }
                else {
                    RRctrl.followLineToSpline(resetPositionB4WarehouseRed);
                    intakeCtrlBlue.setState(IntakeState.BASE);
                    RRctrl.followLineToSpline(resetPositionB4WarehouseRed2);
                    RRctrl.followLineToSpline(warehousePickupPositionRed);
                }
                Log.d("AutonBrain","reset status and init for intake");

                qObjectInRobot.set(false); // reset

                majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                return;
            case THREE_BACK_FORTH:
                doGoBack();
                if(minorState.get().equals(MinorAutonomousState.STOPPED))
                {
                    minorState.set(MinorAutonomousState.ONE_INTAKE);
                    return;
                }
                // time check
               double currentTime = NanoClock.system().seconds();
                if(currentTime- iniTemps > 27) {
                    Log.d("AutonBrain","Time Constraint: parking");
                    majorState.set(MajorAutonomousState.FOUR_PARKING_CLEANUP);
                }
                return;
            case FOUR_PARKING_CLEANUP:
                Log.d("AutonBrain","parking1");
                intakeCtrlBlue.setState(IntakeState.DUMP);
                /*
                Log.d("AutonBrain","parking2");
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                Log.d("AutonBrain","parking3");
                RRctrl.followLineToSpline(parkingPositionBlue);
                Log.d("AutonBrain","parking4");
                */
                //if(!RRctrl.isInWarehouse()) {RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);}
                RRctrl.followLineToSpline(isBlue ? parkingPositionBlue : parkingPositionRed);
                SharedData.autonomousLastPosition = RRctrl.getPos();
                majorState.set(MajorAutonomousState.FINISHED);
                minorState.set(MinorAutonomousState.FINISHED);
                return;
            case FINISHED:
                return;

        }
    }

    public void doGoBack()
    {
        switch(minorState.get()) {
            case STOPPED:
                // No associated action
                return;
            case ONE_INTAKE:
                // step 1: prepare threads
                if (!RRctrl.isInWarehouse())
                {
                    Log.d("AutonBrain","Stuck detected on intake attempt, retrying.");
                    intakeCtrlBlue.setState(IntakeState.PARK);
                    RRctrl.followLineToSpline(new Pose2d(RRctrl.getPos().getX()-5, isBlue ? 70 : -70,isBlue ? 0 : -Math.PI));
                    minorState.set(MinorAutonomousState.FOUR_RETURN_TO_INTAKE);
                    return;
                }
                Log.d("AutonBrain", "Starting intake stage for the " + numberOfTrials + "th time");


                slideCtrl.dumperServo.setPosition(SlideController.dumperPosition_READY);
                new Thread(() -> {
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    boolean isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE);
                    Log.d("AutonBrainThread","status0: qObj " + qObjectInRobot.get() + " qIntake " + intakeCtrlBlue.isObjectInPayload());
                    while(isInState)
                    {
                        Log.d("AutonBrainThread","Status: intakeVal " + intakeCtrlBlue.distanceSensor.getDistance(DistanceUnit.MM) + " x " + RRctrl.getPos().getX());
                        if(intakeCtrlBlue.isObjectInPayload()){
                            Log.d("AutonBrainThread","Found it at x " + RRctrl.getPos().getX());
                            RRctrl.haltTrajectory();
                            qObjectInRobot.set(true);
                            intakeCtrlBlue.setState(IntakeState.DUMP);
                            qObjectIsLoaded.set(true);
                            break;
                        }
                        isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE);
                    }
                    Log.d("AutonBrainThread","status2: qObj " + qObjectInRobot.get() + " qIntake " + intakeCtrlBlue.isObjectInPayload());
                    /*leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    if(qObjectInRobot.get()) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
                        RRctrl.haltTrajectory();
                        RRctrl.stopRobot();
                        new Thread(()->{
                            Log.d("AutonBrainThreadThread","Item Loading");
                            intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly();
                            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            qObjectIsLoaded.set(true);
                            Log.d("AutonBrainThreadThread","Item Loaded");
                        }).start();
                    }
                    else {
                        Log.d("AutonBrainThread","state died, try again soon!");
                    }*/
                }).start();
                // step 2: forward
                Log.d("AutonBrain","Forward init x " + RRctrl.getPos().getX());
                RRctrl.forward(25,velocityIntake,accelIntake);
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                Log.d("AutonBrain","Forward done");
                // step 3: check end conditions.
                numberOfTrials++;
                if(qObjectInRobot.get() || intakeCtrlBlue.isObjectInPayload())
                {
                    if(!qObjectInRobot.get())
                    {
                        Log.d("AutonBrain","That was close....");
                        qObjectInRobot.set(intakeCtrlBlue.isObjectInPayload());
                        new Thread(()->{
                            intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly();
                        }).start();
                    }
                    Log.d("AutonBrain","Proceeding to next stage");
                    minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    return;
                }
                else {
                    Log.d("AutonBrain","no block found, try again.");
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    intakeCtrlBlue.setState(IntakeState.REVERSE);
                    RRctrl.followLineToSpline(warehousePickupPositionBlue);
                    // do a turn here
                    if(isBlue) {
                        RRctrl.turnR(intakeTurnAmount); // fix this val pls thx
                    }
                    else {
                        RRctrl.turnL(intakeTurnAmount); // fix this val pls thx
                    }
                    intakeCtrlBlue.setState(IntakeState.BASE);
                }
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
                */Log.d("AutonBrain","Prepare for drop off");
                if(isBlue) { RRctrl.doBlueDepositTrajectoryNoTurnNonMerged();}
                else {
                    RRctrl.doRedDepositTrajectoryNoTurnNonMerged();
                }// debugging
                Log.d("AutonBrain","Preparation Complete");
                minorState.set(MinorAutonomousState.THREE_DEPOSIT);
                return;
            case THREE_DEPOSIT:
                if (RRctrl.isInWarehouse())
                {
                    Log.d("AutonBrain","Stuck detected on deposit trying, retrying.");
                    RRctrl.followLineToSpline(new Pose2d(RRctrl.getPos().getX()+15,isBlue ? 70 : -70,RRctrl.getPos().getHeading()),100);
                    RRctrl.followLineToSpline(isBlue ? warehousePickupPositionBlue : warehousePickupPositionRed);
                    minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    return;
                }
                if(qObjectIsLoaded.get()) {
                    slideCtrl.extendDropRetract(normalTarget);
                    qObjectInRobot.set(false); // reset
                    qObjectIsLoaded.set(false); // reset
                    Log.d("AutonBrain","Slide drop complete");
                    minorState.set(MinorAutonomousState.FOUR_RETURN_TO_INTAKE);
                }
                return;
            case FOUR_RETURN_TO_INTAKE:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                Log.d("AutonBrain","intake prepped");
                intakeCtrlBlue.setState(IntakeState.BASE);
                if(isBlue) { RRctrl.autonomousPrepAndIntakeFromDepositBlue();}
                else {
                    RRctrl.autonomousPrepAndIntakeFromDepositRed();
                }
                Log.d("AutonBrain","reset status and init for intake");
                minorState.set(MinorAutonomousState.ONE_INTAKE);
                return;

        }
    }

    public static Pose2d startPositionBlue = new Pose2d(14,65.5,0);
    public static Pose2d startPositionRed = new Pose2d(14,-65.5,0); // TODO may need to remeasure
    public static Pose2d warehousePickupPositionBlue = new Pose2d(35,70,0);
    public static Pose2d warehousePickupPositionRed = new Pose2d(35,-70,0);
    public static Pose2d depositPositionBlueNoTurn = new Pose2d(-20,70,0);
    public static Pose2d depositPositionRedNoTurn = new Pose2d(-20,-70,0);
    public static Pose2d depositPositionAllianceBlueTOP = new Pose2d(5.58,64.47, -Math.toRadians(30)); //55
    public static Pose2d depositPositionAllianceRedTOP = new Pose2d(5.58,-64.47, -Math.toRadians(30)); //55
    public static Pose2d depositPositionAllianceBlueMID = new Pose2d(5.58,64.47, -Math.toRadians(31)); //56
    public static Pose2d depositPositionAllianceRedMID = new Pose2d(5.58,-64.47, -Math.toRadians(31)); //56
    public static Pose2d depositPositionAllianceBlueBOT = new Pose2d(5.58,64.47, -Math.toRadians(34));//59
    public static Pose2d depositPositionAllianceRedBOT = new Pose2d(5.58,-64.47, -Math.toRadians(34));//59
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14,75,0);
    public static Pose2d resetPositionB4WarehouseRed = new Pose2d(14,-75,0);
    public static Pose2d resetPositionB4WarehouseBlue2 = new Pose2d(14,70,0);
    public static Pose2d resetPositionB4WarehouseRed2 = new Pose2d(14,-70,0);
    public static Pose2d parkingPositionBlue = new Pose2d(50,70,0);
    public static Pose2d parkingPositionRed = new Pose2d(50,-70,0);
    public static Pose2d whiteLinePosBlue = new Pose2d(29.5,65.5,0);
    public static Pose2d whiteLinePosRed = new Pose2d(29.5,-65.5,0);
    public static double velocityIntake = 22;
    public static double accelIntake = 20;
    public static double intakeTurnAmount = 5; // TODO test and adjust as needed
    //public static double distanceIntake = 40;



}
