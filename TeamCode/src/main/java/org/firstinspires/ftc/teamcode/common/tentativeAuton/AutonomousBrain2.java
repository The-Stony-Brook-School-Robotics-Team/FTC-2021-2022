package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

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
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
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

import java.util.concurrent.atomic.AtomicReference;

@Config
public class AutonomousBrain2 {
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
        ONE_INTAKE_REDO,
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

    public AutonomousBrain2(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) // call in init.
    {
        iniMaxAccel = DriveConstantsMain.MAX_ACCEL;
        iniMaxVel = DriveConstantsMain.MAX_VEL;
        iniMaxAngAccel = DriveConstantsMain.MAX_ANG_ACCEL;
        iniMaxAngVel = DriveConstantsMain.MAX_ANG_VEL;
        DriveConstantsMain.MAX_ACCEL = CustomMaxAccel;
        DriveConstantsMain.MAX_VEL = CustomMaxVel;
        DriveConstantsMain.MAX_ANG_ACCEL = CustomMaxAngAccel;
        DriveConstantsMain.MAX_ANG_VEL = CustomMaxAngVel;
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
                    majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                    minorState.set(MinorAutonomousState.ONE_INTAKE);
                    return;
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
                minorState.set(MinorAutonomousState.ONE_INTAKE);
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
                if(currentTime - iniTemps > 27) {
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
            case ONE_INTAKE_REDO:
                if(qObjectInRobot.get())
                {
                    Log.d("AutonBrain","Oops! I should not be in this state. Continuing!");
                    majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                    minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                }
                Log.d("AutonBrain","Confirmed: No block captured.");
                Log.d("AutonBrain","Intake Redo: init in progress");
                Thread intakeChecker = new Thread(() -> {
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    boolean isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE_REDO);
                    Log.d("AutonBrainThread","status0: qObj " + qObjectInRobot.get() + " qIntake " +  getIntake().isObjectInPayload());
                    while(isInState)
                    {
                        if (getIntake().distanceSensor != null) {
                            Log.d("AutonBrainThread","Status: intakeVal " + getIntake().distanceSensor.getDistance(DistanceUnit.MM) + " x " + RRctrl.getPos().getX());
                        }
                        if(getRightIntakeIsObjectInside()){
                            Log.d("AutonBrainThread","Found it at x " + RRctrl.getPos().getX());
                            qObjectInRobot.set(true);
                            RRctrl.haltTrajectory();
                            getIntake().setState(IntakeState.DUMP);
                            qObjectIsLoaded.set(true);
                            break;
                        }
                        isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE_REDO);
                    }
                    Log.d("AutonBrainThread","status2: qObj " + qObjectInRobot.get() + " qIntake " + getIntake().isObjectInPayload());
                });
                intakeChecker.start();
                Log.d("AutonBrain","Intake Redo: thread launched");
                RRctrl.getDrive().followTrajectorySequence(
                        RRctrl.getDrive().trajectorySequenceBuilder(RRctrl.getPos())
                                .back(12.5)
                                .lineToLinearHeading(PICK_UP_SECONDARY_TRAJECTORY_PICK_UP_BLOCK_POSITION)
                                .build()
                );
                Log.d("AutonBrain","Intake Redo: trajectory ended");
                if(qObjectInRobot.get())
                {
                    Log.d("AutonBrain","Successfully have block. Proceeding to next stage.");
                    majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                    minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                }
                else {
                    intakeChecker.interrupt();
                    Log.d("AutonBrain","No block captured. Trying again.");
                    majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                    minorState.set(MinorAutonomousState.ONE_INTAKE_REDO);
                }
                return;
            case ONE_INTAKE:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                Log.d("AutonBrain","intake prepped");
                if(isBlue){
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
                                qObjectInRobot.set(true);
                                RRctrl.haltTrajectory();
                                getIntake().setState(IntakeState.DUMP);
                                qObjectIsLoaded.set(true);
                                break;
                            }
                            isInState = minorState.get().equals(MinorAutonomousState.ONE_INTAKE);
                        }
                        Log.d("AutonBrainThread","status2: qObj " + qObjectInRobot.get() + " qIntake " + getIntake().isObjectInPayload());
                    }).start();
                    RRctrl.getDrive().followTrajectory(
                            RRctrl.getDrive().trajectoryBuilder(
                                    RRctrl.getDrive().getPoseEstimate())
                                    .lineToSplineHeading(PICK_UP_TRAJECTORY_FIX_HEADING_POSITION)
                                    .splineToConstantHeading(PICK_UP_TRAJECTORY_PASS_PIPE_POSITION, PICK_UP_TRAJECTORY_PASS_PIPE_POSITION_TANGENT)
                                    .splineToConstantHeading(PICK_UP_TRAJECTORY_PICK_UP_POSITION, ZERO)
                                    .addSpatialMarker(PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION, () -> intakeCtrlBlue.setState(IntakeState.BASE))
                                    .build()
                    );
                    if(qObjectInRobot.get())
                    {
                        Log.d("AutonBrain","Successfully have block. Proceeding to next stage.");
                        majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                        minorState.set(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    }
                    else {
                        Log.d("AutonBrain","No block captured. Trying again.");
                        majorState.set(MajorAutonomousState.THREE_BACK_FORTH);
                        minorState.set(MinorAutonomousState.ONE_INTAKE_REDO);
                    }
                    return;
                }
                else {
                    RRctrl.autonomousPrepAndIntakeFromDepositRed();
                }
                Log.d("AutonBrain","reset status and init for intake");
                minorState.set(MinorAutonomousState.ONE_INTAKE);
                return;


            case TWO_PREP_DEPOSIT:
                if(isBlue) {
                    //RRctrl.doBlueDepositTrajectoryNoTurnNonMerged();
                    RRctrl.getDrive().followTrajectory(
                            RRctrl.getDrive().trajectoryBuilder(
                                    RRctrl.getDrive().getPoseEstimate(), true)
                                    .lineToSplineHeading(DEPOSIT_TRAJECTORY_FIX_HEADING_POSITION)
                                    .splineToLinearHeading(DEPOSIT_TRAJECTORY_PASS_PIPE_POSITION, Math.toRadians(-165.0))
                                    .splineToSplineHeading(depositPositionBlueTOP, Math.toRadians(175.0))
                                    .build()
                    );
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
                if(RRctrl.distanceTo(isBlue ? depositPositionAllianceBlueTOP : depositPositionRedNoTurn) > ERROR_TOLERANCE_DROPOFF)
                {
                    Log.d("AutonBrain","Detected not arrived at designated position. Fixing");
                    RRctrl.followLineToSpline(depositPositionBlueTOP);
                    return;
                }
                if(qObjectIsLoaded.get()) {
                    slideCtrl.extendDropRetractAuton(normalTarget);
                    qObjectInRobot.set(false); // reset
                    qObjectIsLoaded.set(false); // reset
                    Log.d("AutonBrain","Slide drop complete");
                    minorState.set(MinorAutonomousState.ONE_INTAKE);
                }
                return;
            case FOUR_RETURN_TO_INTAKE:
                Log.d("AutonBrain","I should not arrive in this state...proceeding to merged intake...");
                minorState.set(MinorAutonomousState.ONE_INTAKE);
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
    public static Pose2d warehousePickupPositionBlue = new Pose2d(43,70,0);
    public static Pose2d warehousePickupPositionRed = new Pose2d(43,-70,-Math.PI);
    public static Pose2d depositPositionBlueNoTurn = new Pose2d(-18,75,0);
    public static Pose2d depositPositionRedNoTurn = new Pose2d(-24,-75,-Math.PI);
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
    public static Pose2d parkingPositionBlue = new Pose2d(65,70,0);
    public static Pose2d parkingPositionRed = new Pose2d(65,-70,-Math.PI);
    public static Pose2d whiteLinePosBlue = new Pose2d(29.5,65.5,0);
    public static Pose2d whiteLinePosRed = new Pose2d(29.5,-65.5,-Math.PI);
    public static double velocityIntake = 50;
    public static double accelIntake = 25;
    public static double intakeTurnAmount = 5; // TODO test and adjust as needed
    //public static double distanceIntake = 40;

    private static final double ANTI_BLOCKING_CHECKER_DEPOSIT_X = 25.0;
    private static final double ANTI_BLOCKING_CHECKER_PICK_UP_X = 40.0;
    private static final double ANTI_BLOCKING_CHECKER_PARK_X = 32.0;
    private static final int ZERO = 0;

    private static final Vector2d PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION = new Vector2d(17.0, 66);
    private static final Pose2d PICK_UP_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(10.0, 65.5, ZERO);
    private static final Vector2d PICK_UP_TRAJECTORY_PASS_PIPE_POSITION = new Vector2d(35.0, 66.0);
    private static final double PICK_UP_TRAJECTORY_PASS_PIPE_POSITION_TANGENT = Math.toRadians(-15.0);
    private static final Vector2d PICK_UP_TRAJECTORY_PICK_UP_POSITION = new Vector2d(58.0, 64.0);

    private static final Pose2d DEPOSIT_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(40.0, 67.0, ZERO);
    private static final Pose2d DEPOSIT_TRAJECTORY_PASS_PIPE_POSITION = new Pose2d(20.0, 65.5, ZERO);   //Heading is identical to B_FIX_HEADING_POSITION

    private static final Pose2d PICK_UP_SECONDARY_TRAJECTORY_PICK_UP_BLOCK_POSITION = new Pose2d(59.0, 64.0, Math.toRadians(0.0));

    public static final Pose2d PARK_TRAJECTORY_PARK_POSITION = new Pose2d(50.0, 66.0, 0);

    private static final Pose2d ABC_RESET_POSITION_PICK_UP = new Pose2d(10.0, 64.0, 0);
    private static final Pose2d ABC_RESET_POSITION_DEPOSIT = new Pose2d(55.0, 64.0, 0.0);
    private static final Pose2d ABC_RESET_POSITION_PARK = new Pose2d(10.0, 64.0, 0);
    private static final Vector2d ABC_CHECK_POSITION_PICK_UP = new Vector2d(50.5, 65.5);
    private static final Vector2d ABC_CHECK_POSITION_DEPOSIT = new Vector2d(12.0, 65.5);
    private static final Vector2d ABC_CHECK_POSITION_PARK = ABC_CHECK_POSITION_PICK_UP;


    public static Pose2d depositPositionBlueTOP = new Pose2d(2.0, 63.0, -Math.toRadians(33.0));
    public static Pose2d depositPositionBlueMID = new Pose2d(2.0, 63.0, -Math.toRadians(34.0));
    public static Pose2d depositPositionBlueBOT = new Pose2d(2.0, 63.0, -Math.toRadians(35.0));


    public static double ERROR_TOLERANCE_DROPOFF;

    public static Pose2d firstDepositPositionBlueTOP = new Pose2d(5.58, 64.47, -Math.toRadians(33.0));
    public static Pose2d firstDepositPositionBlueMID = new Pose2d(5.58, 64.47, -Math.toRadians(31.0));
    public static Pose2d firstDepositPositionBlueBOT = new Pose2d(5.58, 64.47, -Math.toRadians(34.0));
}
