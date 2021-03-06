package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.sharedResources.SharedData;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeController;
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

    double iniMaxAccel;
    double iniMaxVel;
    double iniMaxAngVel;
    double iniMaxAngAccel;


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
        TWO_PLUS_SPLINE_WAREHOUSE,
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

    public static double CustomMaxAccel = 20;
    public static double CustomMaxVel = 70;
    public static double CustomMaxAngAccel = 2;
    public static double CustomMaxAngVel = 2;

    double iniTemps = 0;
    boolean isBlue = true;
    boolean isSpline = true;

    public AutonomousBrain(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) // call in init.
    {
        iniMaxAccel = DriveConstantsMain.MAX_ACCEL;
        iniMaxVel = DriveConstantsMain.MAX_VEL;
        iniMaxAngAccel = DriveConstantsMain.MAX_ANG_ACCEL;
        iniMaxAngVel = DriveConstantsMain.MAX_ANG_VEL;
        //DriveConstantsMain.MAX_ACCEL = CustomMaxAccel;
        //DriveConstantsMain.MAX_VEL = CustomMaxVel;
       // DriveConstantsMain.MAX_ANG_ACCEL = CustomMaxAngAccel;
       // DriveConstantsMain.MAX_ANG_VEL = CustomMaxAngVel;
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
        isSpline = (mode == AutonomousMode.BlueStatesSpline || mode == AutonomousMode.RedStatesSpline);
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
        slideCtrl.blueDumperServo.setPosition(slideCtrl.dumperPosition_CLOSED); // init only
        slideCtrl.redDumperServo.setPosition(slideCtrl.dumperPosition_red_CLOSED); // init only
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
                DriveConstantsMain.MAX_ACCEL = iniMaxAccel;
                DriveConstantsMain.MAX_VEL = iniMaxVel;
                DriveConstantsMain.MAX_ANG_ACCEL = iniMaxAngAccel;
                DriveConstantsMain.MAX_ANG_VEL = iniMaxAngVel;
                return;
            case FINISHED:
                DriveConstantsMain.MAX_ACCEL = iniMaxAccel;
                DriveConstantsMain.MAX_VEL = iniMaxVel;
                DriveConstantsMain.MAX_ANG_ACCEL = iniMaxAngAccel;
                DriveConstantsMain.MAX_ANG_VEL = iniMaxAngVel;
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
                slideCtrl.redDumperServo.setPosition(SlideController.dumperPosition_red_READY);
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
                RRctrl.forward(isBlue ? 25 : -25,velocityIntake,accelIntake);
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

               /*/* new Thread(()->{
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
                *//*Log.d("AutonBrain","Prepare for drop off");
                if(isBlue) {
                    RRctrl.doBlueDepositTrajectoryNoTurnNonMerged();
                }
                else {
                    RRctrl.doRedDepositTrajectoryNoTurnNonMerged();
                }// debugging*/

                if(isBlue) {
                    /*RRctrl.getDrive().followTrajectory(
                            RRctrl.getDrive().trajectoryBuilder(
                                    RRctrl.getDrive().getPoseEstimate(), true)
                                    .splineToSplineHeading(DEPOSIT_TRAJECTORY_FIX_HEADING_POSITION, Math.toRadians(10))
                                    .splineToLinearHeading(DEPOSIT_TRAJECTORY_PASS_PIPE_POSITION, Math.toRadians(-10))
                                    .splineToSplineHeading(firstDepositPositionBlueTOP, Math.toRadians(175.0))
                                    //.addSpatialMarker(DEPOSIT_TRAJECTORY_START_EXTEND_SLIDE_POSITION, this::extendDropRetract_TOP)
                                    //.addDisplacementMarker(this::AntiBlockingChecker_Deposit)
                                    .build()
                    );*/
                    RRctrl.doBlueDepositTrajectoryNoTurnNonMerged();
                }
                else {
                    RRctrl.doRedDepositTrajectoryNoTurnNonMerged();
                }
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
                if(isBlue) {

                   /* RRctrl.getDrive().followTrajectory(
                            RRctrl.getDrive().trajectoryBuilder(
                                    RRctrl.getDrive().getPoseEstimate())
                                    .lineToSplineHeading(PICK_UP_TRAJECTORY_FIX_HEADING_POSITION)
                                    .splineToConstantHeading(PICK_UP_TRAJECTORY_PASS_PIPE_POSITION, PICK_UP_TRAJECTORY_PASS_PIPE_POSITION_TANGENT)
                                    .splineToConstantHeading(PICK_UP_TRAJECTORY_MOVE_OUT_POSITION, PICK_UP_TRAJECTORY_MOVE_OUT_POSITION_TANGENT)
                                    .splineToConstantHeading(RoadRunnerController.convertPose2Vector(warehousePickupPositionBlue), 0)
                                    //.addSpatialMarker(PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION, () -> intakeCtrlBlue.setState(IntakeState.BASE))
                                    .build()
                    );
                    RRctrl.getDrive().update();
*/
                    RRctrl.autonomousPrepAndIntakeFromDepositBlue();
                }
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
    public static double startPositionRedY = -67;
    public static double startPositionRedH = 180;

    public static Pose2d startPositionBlue = new Pose2d(startPositionBlueX,startPositionBlueY,startPositionBlueH);
    public static Pose2d startPositionRed = new Pose2d(startPositionRedX,startPositionRedY,Math.toRadians(startPositionRedH)); // TODO may need to remeasure
    public static Pose2d warehousePickupPositionBlue = new Pose2d(43,65.5,0);
    public static Pose2d warehousePickupPositionRed = new Pose2d(43,-70,-Math.PI);
    public static Pose2d depositPositionBlueNoTurn = new Pose2d(-18,75,0);
    public static Pose2d depositPositionRedNoTurn = new Pose2d(-20,-75,-Math.PI);
    public static Pose2d depositPositionAllianceBlueTOP = new Pose2d(5.58,64.47, -Math.toRadians(30)); //55
    public static Pose2d depositPositionAllianceRedTOP = new Pose2d(5.58,-64.47, -Math.toRadians(150)); //55
    public static Pose2d depositPositionAllianceBlueMID = new Pose2d(5.58,64.47, -Math.toRadians(31)); //56
    public static Pose2d depositPositionAllianceRedMID = new Pose2d(5.58,-64.47, -Math.toRadians(149)); //56
    public static Pose2d depositPositionAllianceBlueBOT = new Pose2d(5.58,64.47, -Math.toRadians(34));//59
    public static Pose2d depositPositionAllianceRedBOT = new Pose2d(5.58,-64.47, -Math.toRadians(145));//59
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14,75,0);
    public static Pose2d resetPositionB4WarehouseRed = new Pose2d(14,-75,-Math.PI);
    public static Pose2d resetPositionB4WarehouseBlue2 = new Pose2d(14,65.5,0);
    public static Pose2d resetPositionB4WarehouseRed2 = new Pose2d(14,-70,-Math.PI);
    public static Pose2d parkingPositionBlue = new Pose2d(65,70,0);
    public static Pose2d parkingPositionRed = new Pose2d(65,-70,-Math.PI);
    public static Pose2d whiteLinePosBlue = new Pose2d(29.5,65.5,0);
    public static Pose2d whiteLinePosRed = new Pose2d(29.5,-65.5,-Math.PI);
    public static double velocityIntake = 50;
    public static double accelIntake = 25;
    public static double intakeTurnAmount = 5; // TODO test and adjust as needed
    //public static double distanceIntake = 40;

    private static final Pose2d DEPOSIT_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(40.0, 67.0, 0);
    private static final Pose2d DEPOSIT_TRAJECTORY_PASS_PIPE_POSITION = new Pose2d(20.0, 68.0, 0);   //Heading is identical to B_FIX_HEADING_POSITION
    private static final Pose2d PICK_UP_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(18.0, 66.0, 0);
    private static final Vector2d PICK_UP_TRAJECTORY_PASS_PIPE_POSITION = new Vector2d(37.0, 66.0);
    private static final double PICK_UP_TRAJECTORY_PASS_PIPE_POSITION_TANGENT = Math.toRadians(-10.0);
    private static final Vector2d PICK_UP_TRAJECTORY_MOVE_OUT_POSITION = new Vector2d(43.0, 64.0);
    private static final double PICK_UP_TRAJECTORY_MOVE_OUT_POSITION_TANGENT = Math.toRadians(-10.0);


    public static Pose2d firstDepositPositionBlueTOP = new Pose2d(5.58, 64.47, -Math.toRadians(33.0));
    public static Pose2d firstDepositPositionBlueMID = new Pose2d(5.58, 64.47, -Math.toRadians(31.0));
    public static Pose2d firstDepositPositionBlueBOT = new Pose2d(5.58, 64.47, -Math.toRadians(34.0));
}
