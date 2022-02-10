package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.sharedResources.SharedData;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
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

    RevBlinkinLedDriver leds;
    NormalizedColorSensor normalizedColorSensor;

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
        THREE_BACK_FORTH,
        FOUR_PARKING_CLEANUP,
        FINISHED
    }
    enum MinorAutonomousState {
        STOPPED,
        ONE_INTAKE,
        TWO_PREP_DEPOSIT,
        THREE_DEPOSIT,
        FOUR_RETURN_TO_INTAKE
    }

    double iniTemps = 0;

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
        this.leds = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");

        RRctrl.setPos(startPositionBlue);
        intakeCtrlBlue.setState(IntakeState.PARK);
        intakeCtrlRed.setState(IntakeState.PARK); // to prevent from moving around
        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        normalizedColorSensor.setGain(Configuration.colorSensorGain);
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

                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.autonomousPrepareForPickup();
                Log.d("AutonBrain","reset status and init for intake");

                qObjectInRobot = false; // reset

                majorState = MajorAutonomousState.THREE_BACK_FORTH;
                return;
            case THREE_BACK_FORTH:
                doGoBack();
                if(minorState == MinorAutonomousState.STOPPED)
                {
                    minorState = MinorAutonomousState.ONE_INTAKE;
                    return;
                }
                // time check
               double currentTime = NanoClock.system().seconds();
                if(currentTime- iniTemps > 35) {
                    Log.d("AutonBrain","Time Constraint: parking");
                    majorState = MajorAutonomousState.FOUR_PARKING_CLEANUP;
                }
                return;
            case FOUR_PARKING_CLEANUP:
                Log.d("AutonBrain","parking1");
                intakeCtrlBlue.setState(IntakeState.PARK);
                /*
                Log.d("AutonBrain","parking2");
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                Log.d("AutonBrain","parking3");
                RRctrl.followLineToSpline(parkingPositionBlue);
                Log.d("AutonBrain","parking4");
                */
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                RRctrl.followLineToSpline(parkingPositionBlue);
                SharedData.autonomousLastPosition = RRctrl.getPos();
                majorState = MajorAutonomousState.FINISHED;
                return;
            case FINISHED:
                return;

        }
    }

    public void doGoBack()
    {
        switch(minorState)
        {
            case STOPPED:
                // No associated action
                return;
            case ONE_INTAKE:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                Log.d("AutonBrain","Current Status: itemBool: " + qObjectInRobot + " intakeStatus " + intakeCtrlBlue.isObjectInPayload());
                if(qObjectInRobot || intakeCtrlBlue.isObjectInPayload())
                {
                    //We have a block
                    Log.d("AutonBrain","Missed block on last run, proceeding.");
                    minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
                    return;
                }

                intakeCtrlBlue.setState(IntakeState.BASE);
                new Thread(()->{
                    boolean isInState = minorState.equals(MinorAutonomousState.ONE_INTAKE);
                    while(isInState)
                    {
                        isInState = minorState.equals(MinorAutonomousState.ONE_INTAKE);
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
                        Log.d("AutonBrainThread","Status: scoop: " + qObjectInRobot +" state " + isInState);
                    }
                    Log.d("AutonBrainThread","Status2: scoop: " + qObjectInRobot +" state " + isInState);
                    if(qObjectInRobot)
                    {
                        RRctrl.stopTrajectory();
                        intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly();
                        Log.d("AutonBrainThread","Status: loaded");
                    }
                }).start();
                Log.d("AutonBrain","Forward init");
                RRctrl.forward(20,velocityIntake);
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                Log.d("AutonBrain","Forward done");
                RRctrl.stopRobot();
                RRctrl.stopRobot();
                // stopped
                if(qObjectInRobot)
                {
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
                    Log.d("AutonBrain","Continuing to deposit");
                    return;
                }
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                RRctrl.followLineToSpline(depositPrepPositionBlue);
                Log.d("AutonBrain","Retrying to find a block");
                return;
            case TWO_PREP_DEPOSIT: // TODO implement go forward and then turn
                /*RRctrl.followLineToSpline(startPositionBlue);
                RRctrl.followLineToSpline(depositPositionAllianceBlue);
                */
                new Thread(()->{
                    // do white line repositioning
                    boolean isInState = minorState.equals(MinorAutonomousState.TWO_PREP_DEPOSIT);
                    while(isInState)
                    {
                        isInState = minorState.equals(MinorAutonomousState.TWO_PREP_DEPOSIT);
                        if(normalizedColorSensor.getNormalizedColors().alpha > Configuration.colorSensorWhiteAlpha)
                        {
                            RRctrl.setPos(whiteLinePos);
                            break; // done.
                        }
                    }
                }).start();
                Log.d("AutonBrain","Prepare for drop off");
                //RRctrl.doBlueDepositTrajectory();
                RRctrl.doBlueDepositTrajectoryNoTurn(); // debugging
                Log.d("AutonBrain","Preparation Complete");
                minorState = MinorAutonomousState.THREE_DEPOSIT;
                return;
            case THREE_DEPOSIT:
                slideCtrl.extendDropRetract(normalTarget);
                Log.d("AutonBrain","Slide drop complete");
                minorState = MinorAutonomousState.FOUR_RETURN_TO_INTAKE;
                return;
            case FOUR_RETURN_TO_INTAKE:
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                RRctrl.setPos(new Pose2d(resetPositionB4WarehouseBlue.getX(),65.5,0)); // reset contre mur.
                Log.d("AutonBrain","intake prepped");
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.autonomousPrepareForPickup();
                Log.d("AutonBrain","reset status and init for intake");
                qObjectInRobot = false; // reset
                minorState = MinorAutonomousState.ONE_INTAKE;
                return;

        }
    }

    public static Pose2d startPositionBlue = new Pose2d(14,65.5,0);
    public static Pose2d warehousePickupPositionBlue = new Pose2d(35,70,0);
    public static Pose2d depositPrepPositionBlue = new Pose2d(30,70,0);
    public static Pose2d depositPrepPositionBlueNoTurn = new Pose2d(-20,70,0);
    public static Pose2d depositPositionAllianceBlueTOP = new Pose2d(5.58,64.47,-Math.toRadians(55));
    public static Pose2d depositPositionAllianceBlueMID = new Pose2d(5.58,64.47,-Math.toRadians(56));
    public static Pose2d depositPositionAllianceBlueBOT = new Pose2d(5.58,64.47,-Math.toRadians(59));
    public static Pose2d depositPositionAllianceBlue2 = new Pose2d(5.58,64.47,-Math.toRadians(48));
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14,75,0);
    public static Pose2d parkingPositionBlue = new Pose2d(45,75,0);
    public static Pose2d whiteLinePos = new Pose2d(28.5,65.5,0);
    public static double velocityIntake = 15;


}
