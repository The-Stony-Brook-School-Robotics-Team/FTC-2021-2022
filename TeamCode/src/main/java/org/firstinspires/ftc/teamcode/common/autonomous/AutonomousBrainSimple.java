package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.sharedResources.SharedData;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.ParkingProbingSensorController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;
import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class AutonomousBrainSimple {
    public static double EPSILON_DIST = 3; // in
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideController slideCtrl;
    IntakeControllerBlue intakeCtrlBlue;
    IntakeControllerRed intakeCtrlRed;
    DuckCarouselController duckCtrl;
    ParkingProbingSensorController parkingProber;
    Pose2d iniDropPosition = depositPositionAllianceBlueTOP;



    boolean isParkingAvailable = false;
    int numberOfTrials = 1;
    RevBlinkinLedDriver leds;
    //NormalizedColorSensor normalizedColorSensor;
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
        ONE_CAMERA_READ_CAROUSEL,
        TWO_DEPOSIT_INI_BLOCK,
        TWO_DEPOSIT_INI_BLOCK_CAROUSEL,
        TWO_PLUS_SPLINE_WAREHOUSE,
        THREE_BACK_FORTH,
        THREE_WAIT_FOR_TIMER,
        FOUR_PARKING_CLEANUP,
        FOUR_GO_SLOWLY_TO_PARK_WAREHOUSE,
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
    boolean isSpline = true;

    public AutonomousBrainSimple(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) // call in init.
    {
        DriveConstantsMain.MAX_ACCEL = 20;
        DriveConstantsMain.MAX_VEL = 100;
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
        this.parkingProber = robot.getParkProber();
        this.leds = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
        isBlue = (mode == AutonomousMode.BlueStatesWarehouse || mode == AutonomousMode.BlueStatesSpline || mode == AutonomousMode.BlueStatesDuckSimple);
        isSpline = (mode == AutonomousMode.BlueStatesSpline || mode == AutonomousMode.RedStatesSpline);
        if (isBlue) {
            if(!isSpline) {
                RRctrl.setPos(startPositionBlueCarousel);
            }
            else {
                RRctrl.setPos(startPositionBlue);
            }
        } else {
            if(!isSpline) {
               //
            }
            else {
                RRctrl.setPos(startPositionRed);
            }
        }
        if(mode == AutonomousMode.RedStatesDuckSimple)
        {
            RRctrl.setPos(startPositionRedCarousel);
            isBlue = false;
            isSpline = false;
        }
        intakeCtrlBlue.setState(IntakeState.DUMP);
        intakeCtrlRed.setState(IntakeState.DUMP); // to prevent from moving around
        //normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        colorNew = hardwareMap.get(RevColorSensorV3.class, "color");
        colorNew.write8(BroadcomColorSensor.Register.LS_MEAS_RATE,0x01010000); // see pdf page 20 // increase speed
        colorNew.write8(BroadcomColorSensor.Register.PS_MEAS_RATE,0x00000001); // see pdf page 19 // increase speed
        slideCtrl.blueDumperServo.setPosition(slideCtrl.dumperPosition_CLOSED); // init only
        slideCtrl.redDumperServo.setPosition(slideCtrl.dumperPosition_CLOSED); // init only
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
                if(mode == AutonomousMode.BlueStatesDuckSimple || mode == AutonomousMode.RedStatesDuckSimple) {
                    majorState.set(MajorAutonomousState.ONE_CAMERA_READ_CAROUSEL);
                    return;
                }
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
            case ONE_CAMERA_READ_CAROUSEL:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ", heightFromDuck.toString());
                CVctrl.shutDown();
                switch (heightFromDuck) {
                    case ONE:
                        iniTarget = SlideTarget.BOTTOM_CAROUSEL;
                        iniDropPosition = isBlue ? duckSpinningPositionB1 : duckSpinningPositionR1;
                        break;
                    case TWO:
                        iniTarget = SlideTarget.MIDDLE_CAROUSEL;
                        iniDropPosition = isBlue ? duckSpinningPositionB2 : duckSpinningPositionR2;
                        break;
                    case THREE:
                        iniTarget = SlideTarget.TOP_CAROUSEL;
                        iniDropPosition = isBlue ? duckSpinningPositionB : duckSpinningPositionR;
                        break;
                }
                majorState.set(MajorAutonomousState.TWO_DEPOSIT_INI_BLOCK_CAROUSEL);
                return;
            case TWO_DEPOSIT_INI_BLOCK:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                RRctrl.followLineToSpline(iniDropPosition);
                slideCtrl.extendDropRetractAuton(iniTarget);
                Log.d("AutonBrain","Slide drop complete");
                if(isBlue) {
                    RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                    intakeCtrlBlue.setState(IntakeState.BASE);
                    intakeCtrlRed.setState(IntakeState.PARK);
                    RRctrl.followLineToSpline(resetPositionB4WarehouseBlue2);
                    RRctrl.followLineToSpline(warehousePickupPositionBlue);
                }
                else {
                    RRctrl.followLineToSpline(resetPositionB4WarehouseRed);
                    intakeCtrlRed.setState(IntakeState.BASE);
                    intakeCtrlBlue.setState(IntakeState.PARK);
                    RRctrl.followLineToSpline(resetPositionB4WarehouseRed2);
                    RRctrl.followLineToSpline(warehousePickupPositionRed);
                }
                Log.d("AutonBrain","reset status and init for intake");

                qObjectInRobot.set(false); // reset
                if(isSpline) {
                    majorState.set(MajorAutonomousState.TWO_PLUS_SPLINE_WAREHOUSE);
                    return;
                }
                majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                return;
            case TWO_PLUS_SPLINE_WAREHOUSE:
                RRctrl.followSplineTrajWarehouse(true);
                return;
            case TWO_DEPOSIT_INI_BLOCK_CAROUSEL:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
                RRctrl.followLineToSpline(iniDropPosition);
                new Thread(()->{
                    slideCtrl.extendDropRetractAuton(iniTarget);
                    Log.d("AutonBrain","Slide drop complete");
                }).start();
                robot.getDuckCtrl().spinOneDuck(isBlue);
                Log.d("AutonBrain","Duck spin complete");
                majorState.set(MajorAutonomousState.THREE_WAIT_FOR_TIMER);
                return;
            case THREE_WAIT_FOR_TIMER:
                double currentTime2 = NanoClock.system().seconds();
                if(currentTime2 - iniTemps > 20) { // ten seconds left
                    Log.d("AutonBrain","Time Constraint: adaptive parking");
                    majorState.set(MajorAutonomousState.FOUR_GO_SLOWLY_TO_PARK_WAREHOUSE);
                }
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
                intakeCtrlRed.setState(IntakeState.DUMP);
                RRctrl.followLineToSpline(isBlue ? parkingPositionBlue : parkingPositionRed);
                SharedData.autonomousLastPosition = RRctrl.getPos();
                majorState.set(MajorAutonomousState.FINISHED);
                minorState.set(MinorAutonomousState.FINISHED);
                return;
            case FOUR_GO_SLOWLY_TO_PARK_WAREHOUSE:
                /*doAnalysisMaster = true;
                if(CVctrl.prepareWhiteLineEngine()) {
                    Sleep.sleep(1000);
                    isParkingAvailable = false; // TODO CVctrl.getWhiteLineAvailable();
                }
                doAnalysisMaster = false;
                CVctrl.shutDown();*/
                AtomicReference<Boolean> hasFinishedTraj = new AtomicReference<>();
                AtomicReference<Boolean> hasHaltedTraj = new AtomicReference<>();
                hasFinishedTraj.set(false);
                hasHaltedTraj.set(false);
                new Thread(()->{
                    while(!hasFinishedTraj.get())
                    {
                        if(!parkingProber.isOpeningAvailableAuton(isBlue,RRctrl.distanceTo(isBlue ? parkingPositionBlue : parkingPositionRed)))
                        {
                            hasHaltedTraj.set(true);
                            Log.d("AutonBrain","[ATTENTION] Halted Parking attempt: robot detected.");
                            Log.e("AutonBrain","[ATTENTION] Halted Parking attempt: robot detected.");
                            break; // AAAAAH!! STOP!!
                        }
                    }
                    RRctrl.endTrajectory(); // KILL!!!
                    hasFinishedTraj.set(true);
                }).start();
                if(!isBlue) {
                    RRctrl.simpleAutonParkingProbeRed();
                }
                else {
                    RRctrl.simpleAutonParkingProbeBlue();
                }
                hasFinishedTraj.set(true);
                if(hasHaltedTraj.get())
                {
                    // backpedal fast!!!!
                    TrajectoryVelocityConstraint velocityConstraintFast = SampleMecanumDrive.getVelocityConstraint(100, DriveConstantsMain.MAX_ANG_VEL, DriveConstantsMain.TRACK_WIDTH);
                    TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(80);
                    RRctrl.followLineToSpline((isBlue ? parkingPositionBlueStorageUnit : parkingPositionRedStorageUnit), velocityConstraintFast,accelerationConstraint);
                    try
                    {
                        RRctrl.followLineToSpline((isBlue ? parkingPositionBlueStorageUnit : parkingPositionRedStorageUnit));
                    }
                    catch (EmptyPathSegmentException e)
                    {
                       Log.d("AutonBrain","Looks like we are in place!");
                    }
                    catch(Exception e)
                    {
                        Log.d("AutonBrain","Funky Excpetion thrown...\n" +e.getMessage());
                    }
                    Log.d("AutonBrain","Target to current delta: " + RRctrl.distanceTo(parkingPositionBlueStorageUnit));
                    majorState.set(MajorAutonomousState.FINISHED);
                    Log.d("AutonBrain","finished all tasks.");
                    // done!
                    return;
                }
                double dist = RRctrl.distanceTo(parkingPositionBlue);
                Log.d("AutonBrain","Target to current delta: " + dist);
                if(dist > EPSILON_DIST)
                {
                    Log.d("AutonBrain","Too far: need to correct");
                    RRctrl.followLineToSpline(parkingPositionBlue);
                    Log.d("AutonBrain","Corrected.");
                    Log.d("AutonBrain","Target to current delta: " + dist);
                }
                majorState.set(MajorAutonomousState.FINISHED);
                Log.d("AutonBrain","finished all tasks.");
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
                if (!RRctrl.isInWarehouse(isBlue))
                {
                    Log.d("AutonBrain","Stuck detected on intake attempt, retrying.");
                    getIntake().setState(IntakeState.PARK);
                    //RRctrl.backward(5);
                    RRctrl.followLineToSpline(new Pose2d(RRctrl.getPos().getX()-5,70,0));
                    RRctrl.followLineToSpline(warehousePickupPositionBlue);
                    getIntake().setState(IntakeState.BASE);
                    //minorState.set(MinorAutonomousState.FOUR_RETURN_TO_INTAKE);
                    return;
                }
                Log.d("AutonBrain", "Starting intake stage for the " + numberOfTrials + "th time");


                slideCtrl.blueDumperServo.setPosition(SlideController.dumperPosition_READY);
                slideCtrl.redDumperServo.setPosition(SlideController.dumperPosition_READY);
                new Thread(() -> {
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    boolean isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE);
                    Log.d("AutonBrainThread","status0: qObj " + qObjectInRobot.get() + " qIntake " +  getIntake().isObjectInPayload());
                    while(isInState)
                    {
                        if (getIntake().distanceSensor != null) {
                            Log.d("AutonBrainThread","Status: intakeVal " + getIntake().distanceSensor.getDistance(DistanceUnit.MM) + " x " + RRctrl.getPos().getX());
                        }
                        if(getRightIntakeIsObjectInside()){
                            Log.d("AutonBrainThread","Found it at x " + RRctrl.getPos().getX());
                            RRctrl.haltTrajectory();
                            qObjectInRobot.set(true);
                            getIntake().setState(IntakeState.DUMP);
                            qObjectIsLoaded.set(true);
                            break;
                        }
                        isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE);
                    }
                    Log.d("AutonBrainThread","status2: qObj " + qObjectInRobot.get() + " qIntake " + getIntake().isObjectInPayload());
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
                RRctrl.forward(isBlue ? distanceIntake : -1*distanceIntake,velocityIntake,accelIntake);
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                Log.d("AutonBrain","Forward done");
                // step 3: check end conditions.
                numberOfTrials++;
                if(qObjectInRobot.get() || getIntake().isObjectInPayload())
                {
                    if(!qObjectInRobot.get())
                    {
                        Log.d("AutonBrain","That was close....");
                        qObjectInRobot.set(getIntake().isObjectInPayload());
                        new Thread(()->{
                            getIntake().loadItemIntoSlideForAutonomousOnly();
                        }).start();
                    }
                    Log.d("AutonBrain","Proceeding to next stage");
                    minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    return;
                }
                else {
                    Log.d("AutonBrain","no block found, try again.");
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    getIntake().setState(IntakeState.REVERSE);
                    RRctrl.followLineToSpline(isBlue ? warehousePickupPositionBlue : warehousePickupPositionRed);
                    // do a turn here
                    if(isBlue) {
                        RRctrl.turnR(intakeTurnAmount); // fix this val pls thx
                    }
                    else {
                        RRctrl.turnL(intakeTurnAmount); // fix this val pls thx
                    }
                    getIntake().setState(IntakeState.BASE);
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
                if(isBlue) {
                    RRctrl.doBlueDepositTrajectoryNoTurnNonMerged();
                }
                else {
                    RRctrl.doRedDepositTrajectoryNoTurnNonMerged();
                }// debugging
                Log.d("AutonBrain","Preparation Complete");
                minorState.set(MinorAutonomousState.THREE_DEPOSIT);
                return;
            case THREE_DEPOSIT:
                if (RRctrl.isInWarehouse(isBlue))
                {
                    Log.d("AutonBrain","Stuck detected on deposit trying, retrying.");
                    RRctrl.followLineToSpline(new Pose2d(RRctrl.getPos().getX()+ (isBlue ? 15 : -15),isBlue ? 70 : -70,RRctrl.getPos().getHeading()),100);
                    RRctrl.followLineToSpline(isBlue ? warehousePickupPositionBlue : warehousePickupPositionRed);
                    minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    return;
                }
                if(qObjectIsLoaded.get()) {
                    slideCtrl.extendDropRetractAuton(normalTarget);
                    qObjectInRobot.set(false); // reset
                    qObjectIsLoaded.set(false); // reset
                    Log.d("AutonBrain","Slide drop complete");
                    minorState.set(MinorAutonomousState.FOUR_RETURN_TO_INTAKE);
                }
                return;
            case FOUR_RETURN_TO_INTAKE:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                Log.d("AutonBrain","intake prepped");
                getIntake().setState(IntakeState.BASE);
                if(isBlue) { RRctrl.autonomousPrepAndIntakeFromDepositBlue();}
                else {
                    RRctrl.autonomousPrepAndIntakeFromDepositRed();
                }
                Log.d("AutonBrain","reset status and init for intake");
                minorState.set(MinorAutonomousState.ONE_INTAKE);
                return;

        }
    }

    private boolean getRightIntakeIsObjectInside() {
        return isBlue ? intakeCtrlBlue.isObjectInPayload() : intakeCtrlRed.isObjectInPayload();
    }
    private IntakeController getIntake()
    {
        return isBlue ? intakeCtrlBlue : intakeCtrlRed;
    }



    public static double startPositionBlueX = 14;
    public static double startPositionBlueY = 65.5;
    public static double startPositionBlueH = 0;
    public static double startPositionRedX = 14;
    public static double startPositionRedY = -65.5;
    public static double startPositionRedH = 180;

    public static Pose2d startPositionBlue = new Pose2d(startPositionBlueX,startPositionBlueY,startPositionBlueH);
    public static Pose2d startPositionRed = new Pose2d(startPositionRedX,startPositionRedY,Math.toRadians(startPositionRedH)); // TODO may need to remeasure
    public static Pose2d startPositionBlueCarousel = new Pose2d(-42,66,0);
    public static Pose2d startPositionRedCarousel = new Pose2d(-42,-66,Math.PI);
    public static Pose2d warehousePickupPositionBlue = new Pose2d(43,70,0);
    public static Pose2d warehousePickupPositionRed = new Pose2d(43,-70,-Math.PI);
    public static Pose2d depositPositionBlueNoTurn = new Pose2d(-11,75,0);
    public static Pose2d depositPositionRedNoTurn = new Pose2d(-11,-75,-Math.PI);
    public static Pose2d depositPositionAllianceBlueTOP = new Pose2d(5.58,64.47, -Math.toRadians(30)); //55
    public static Pose2d depositPositionAllianceRedTOP = new Pose2d(5.58,-64.47, -Math.toRadians(150)); //55
    public static Pose2d depositPositionAllianceBlueMID = new Pose2d(5.58,64.47, -Math.toRadians(31)); //56
    public static Pose2d depositPositionAllianceRedMID = new Pose2d(5.58,-64.47, -Math.toRadians(149)); //56
    public static Pose2d depositPositionAllianceBlueBOT = new Pose2d(5.58,64.47, -Math.toRadians(34));//59
    public static Pose2d depositPositionAllianceRedBOT = new Pose2d(5.58,-64.47, -Math.toRadians(145));//59
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14,75,0);
    public static Pose2d resetPositionB4WarehouseRed = new Pose2d(14,-75,-Math.PI);
    public static Pose2d resetPositionB4WarehouseBlue2 = new Pose2d(14,70,0);
    public static Pose2d resetPositionB4WarehouseRed2 = new Pose2d(14,-70,-Math.PI);
    public static Pose2d parkingPositionBlue = new Pose2d(50,70,0);
    public static Pose2d parkingPositionRed = new Pose2d(50,-70,-Math.PI);
    public static Pose2d parkingPositionBlueStorageUnit = new Pose2d(-70,36,Math.PI/2);
    public static Pose2d parkingPositionRedStorageUnit = new Pose2d(-70,-36,Math.PI/2);
    public static Pose2d whiteLinePosBlue = new Pose2d(29.5,65.5,0);
    public static Pose2d whiteLinePosRed = new Pose2d(29.5,-65.5,-Math.PI);
    public static double velocityIntake = 30;
    public static double accelIntake = 25;
    public static double intakeTurnAmount = 5; // TODO test and adjust as needed
    public static double distanceIntake = 40;
    public static Pose2d duckSpinningPositionB = new Pose2d(-60, 63, Math.toRadians(46));
    public static Pose2d duckSpinningPositionB2 = new Pose2d(-60, 63, Math.toRadians(45));
    public static Pose2d duckSpinningPositionB1 = new Pose2d(-60, 63, Math.toRadians(43));
    public static Pose2d duckSpinningPositionR = new Pose2d(-60, -63, Math.toRadians(134));
    public static Pose2d duckSpinningPositionR2 = new Pose2d(-60, -63, Math.toRadians(135));
    public static Pose2d duckSpinningPositionR1 = new Pose2d(-60, -63, Math.toRadians(137));
}
