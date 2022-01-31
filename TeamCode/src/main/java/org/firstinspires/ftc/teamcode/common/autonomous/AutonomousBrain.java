package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
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


public class AutonomousBrain {
    AutonomousMode mode;
    Robot robot;
    OpenCVController CVctrl;
    RoadRunnerController RRctrl;
    SlideController slideCtrl;
    IntakeControllerBlue intakeCtrlBlue;
    IntakeControllerRed intakeCtrlRed;
    DuckCarouselController duckCtrl;

    NormalizedColorSensor normalizedColorSensor;

    Telemetry tel;
    HardwareMap hwMap;
    boolean qObjetDansRobot = false;

    public MajorAutonomousState majorState = MajorAutonomousState.STOPPED;
    public MinorAutonomousState minorState = MinorAutonomousState.STOPPED;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;

    SlideTarget iniTarget; // position va le decider. // randomizée
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
        RRctrl.setPos(startPositionBlue);
        intakeCtrlBlue.setState(IntakeState.PARK);
        intakeCtrlRed.setState(IntakeState.PARK); // to prevent from moving around
        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        normalizedColorSensor.setGain(Configuration.colorSensorGain);

    }
    public void lance() // call this method before loop, so start method.
    {
        iniTemps = NanoClock.system().seconds();
    }
    public void faitActionAutonome() // call in loop (once per loop pls)
    {
        switch(majorState) {
            case STOPPED:
                doAnalysisMaster = true;
                majorState = MajorAutonomousState.ONE_CAMERA_READ;
                return;
            case ONE_CAMERA_READ:
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ", heightFromDuck.toString());
                CVctrl.shutDown();
                switch (heightFromDuck) {
                    case ONE:
                        iniTarget = SlideTarget.BOTTOM_DEPOSIT;
                        break;
                    case TWO:
                        iniTarget = SlideTarget.MID_DEPOSIT;
                        break;
                    case THREE:
                        iniTarget = SlideTarget.TOP_DEPOSIT;
                        break;
                }
                majorState = MajorAutonomousState.TWO_DEPOSIT_INI_BLOCK;
                return;
            case TWO_DEPOSIT_INI_BLOCK:
                RRctrl.followLineToSpline(depositPositionAllianceBlue);
                slideCtrl.extendDropRetract(normalTarget);
                Log.d("AutonBrain","Slide drop complete");
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.followLineToSpline(warehousePickupPositionBlue);
                Log.d("AutonBrain","reset status and init for intake");
                qObjetDansRobot = false; // reset
                majorState = MajorAutonomousState.THREE_BACK_FORTH;
                return;
            case THREE_BACK_FORTH:
                faitAllezRetour();
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

    public void faitAllezRetour()
    {
        switch(minorState)
        {
            case STOPPED:
                // fait rien; l'état majeur  va changer l'état si il y a besoin.
                // Pouvez-vous utiliser l'anglais?
                // 你能使用英文吗？
                // CAN YOU SPEAK ENGLISH?
                return;
            case ONE_INTAKE:
                Log.d("AutonBrain","Current Status: itemBool: " + qObjetDansRobot + " intakeStatus " + intakeCtrlBlue.isObjectInPayload());
                if(qObjetDansRobot || intakeCtrlBlue.isObjectInPayload())
                {
                    // nous avons le bloc
                    Log.d("AutonBrain","Missed block on last run, proceeding.");
                    minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
                    return;
                }

                intakeCtrlBlue.setState(IntakeState.BASE);
                new Thread(()->{
                    boolean isInState = minorState.equals(MinorAutonomousState.ONE_INTAKE);
                    Log.d("AutonBrainThread","Status0: scoop: " + qObjetDansRobot +" state " + isInState);
                    while(!qObjetDansRobot && isInState)
                    {
                        Sleep.sleep(10);
                        isInState = minorState.equals(MinorAutonomousState.ONE_INTAKE);
                        qObjetDansRobot = intakeCtrlBlue.isObjectInPayload();
                        Log.d("AutonBrainThread","Status: scoop: " + qObjetDansRobot +" state " + isInState);
                    }
                    Log.d("AutonBrainThread","Status2: scoop: " + qObjetDansRobot +" state " + isInState);
                    if(qObjetDansRobot)
                    {
                        RRctrl.stopTrajectory();
                        intakeCtrlBlue.loadItemIntoSlideForAutonomousOnly();
                        Log.d("AutonBrainThread","Status: loaded");
                    }
                }).start();
                Log.d("AutonBrain","Forward init");
                RRctrl.forward(40,10);
                Log.d("AutonBrain","Forward done");
                RRctrl.stopRobot();
                RRctrl.stopRobot();
                // stopped
                if(qObjetDansRobot)
                {
                    minorState = MinorAutonomousState.TWO_PREP_DEPOSIT;
                    Log.d("AutonBrain","Continuing to deposit");
                    return;
                }
                RRctrl.followLineToSpline(warehousePickupPositionBlue);
                Log.d("AutonBrain","Retrying to find a block");
                return;
            case TWO_PREP_DEPOSIT: // TODO implement go forward and then turn
                /*RRctrl.followLineToSpline(startPositionBlue);
                RRctrl.followLineToSpline(depositPositionAllianceBlue);
                */
                RRctrl.doBlueDepositTrajectory();
                Log.d("AutonBrain","Prepare for drop off");
                minorState = MinorAutonomousState.THREE_DEPOSIT;
                return;
            case THREE_DEPOSIT:
                slideCtrl.extendDropRetract(normalTarget);
                Log.d("AutonBrain","Slide drop complete");
                minorState = MinorAutonomousState.FOUR_RETURN_TO_INTAKE;
                return;
            case FOUR_RETURN_TO_INTAKE:
                RRctrl.followLineToSpline(resetPositionB4WarehouseBlue);
                RRctrl.setPos(new Pose2d(resetPositionB4WarehouseBlue.getX(),65.5,0)); // reset contre mur.
                Log.d("AutonBrain","intake prepped");
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.followLineToSpline(warehousePickupPositionBlue);
                Log.d("AutonBrain","reset status and init for intake");
                qObjetDansRobot = false; // reset
                minorState = MinorAutonomousState.ONE_INTAKE;
                return;

        }
    }

    public static Pose2d startPositionBlue = new Pose2d(14,65.5,0);
    public static Pose2d warehousePickupPositionBlue = new Pose2d(35,65.5,0);
    public static Pose2d depositPositionAllianceBlue = new Pose2d(5.58,64.47,-Math.toRadians(57));
    public static Pose2d depositPositionAllianceBlue2 = new Pose2d(5.58,64.47,-Math.toRadians(56));
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14,80,0);
    public static Pose2d parkingPositionBlue = new Pose2d(60,80,0);


}
