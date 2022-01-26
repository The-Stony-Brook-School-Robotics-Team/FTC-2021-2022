package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public ÉtatsAutonomesMajeurs etatMajeur = ÉtatsAutonomesMajeurs.ARRÊTÉ;
    public ÉtatsAutonomesMineurs minorState = ÉtatsAutonomesMineurs.ARRÊTÉ;
    TowerHeightFromDuck heightFromDuck = TowerHeightFromDuck.NOT_YET_SET;

    SlideTarget butObjectInitiale; // position va le decider. // randomizée
    SlideTarget butNormale = SlideTarget.TOP_DEPOSIT;

    enum ÉtatsAutonomesMajeurs {
        ARRÊTÉ,
        UN_LIT_POSITION_DU_CANARD,
        DEUX_TOURNE_DEPOSE_OBJET,
        TROIS_ALLEZ_RETOUR,
        QUATRE_PARKING_DANS_LE_WAREHOUSE,
        FINIT
    }
    enum ÉtatsAutonomesMineurs {
        ARRÊTÉ,
        UN_ACQUISITION,
        DEUX_MOUVEMENT_VERS_DEPOSITION,
        TROIS_DEPOSITION,
        QUATRE_RETOUR_POUR_ACQUISITION
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
        RRctrl.setPos(positionDépartBleu);
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
        switch(etatMajeur) {
            case ARRÊTÉ:
                doAnalysisMaster = true;
                etatMajeur = ÉtatsAutonomesMajeurs.UN_LIT_POSITION_DU_CANARD;
                return;
            case UN_LIT_POSITION_DU_CANARD:
                heightFromDuck = CVctrl.getWhichTowerHeight();
                Log.d("height: ", heightFromDuck.toString());
                CVctrl.shutDown();
                switch (heightFromDuck) {
                    case ONE:
                        butObjectInitiale = SlideTarget.BOTTOM_DEPOSIT;
                        break;
                    case TWO:
                        butObjectInitiale = SlideTarget.MID_DEPOSIT;
                        break;
                    case THREE:
                        butObjectInitiale = SlideTarget.TOP_DEPOSIT;
                        break;
                }
                etatMajeur = ÉtatsAutonomesMajeurs.DEUX_TOURNE_DEPOSE_OBJET;
                return;
            case DEUX_TOURNE_DEPOSE_OBJET:
                RRctrl.followLineToSpline(positionDeposerBlocSurTourBleu);
                slideCtrl.extendDropRetract(butNormale);
                Log.d("AutonBrain","Slide drop complete");
                RRctrl.followLineToSpline(positionContreMurAvantWarehouseBleu);
                RRctrl.setPos(new Pose2d(14,65.5,0));
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.followLineToSpline(positionCollectionDObjetBleu);
                Log.d("AutonBrain","reset status and init for intake");
                qObjetDansRobot = false; // reset
                etatMajeur = ÉtatsAutonomesMajeurs.TROIS_ALLEZ_RETOUR;
                return;
            case TROIS_ALLEZ_RETOUR:
                faitAllezRetour();
                if(minorState == ÉtatsAutonomesMineurs.ARRÊTÉ)
                {
                    minorState = ÉtatsAutonomesMineurs.UN_ACQUISITION;
                    return;
                }
                // time check
               double currentTime = NanoClock.system().seconds();
                if(currentTime- iniTemps > 35) {
                    Log.d("AutonBrain","Time Constraint: parking");
                    etatMajeur = ÉtatsAutonomesMajeurs.QUATRE_PARKING_DANS_LE_WAREHOUSE;
                }
                return;
            case QUATRE_PARKING_DANS_LE_WAREHOUSE:
                Log.d("AutonBrain","parking1");
                intakeCtrlBlue.setState(IntakeState.PARK);
                Log.d("AutonBrain","parking2");
                RRctrl.followLineToSpline(positionContreMurAvantWarehouseBleu);
                Log.d("AutonBrain","parking3");
                RRctrl.followLineToSpline(positionFinaleBleu);
                Log.d("AutonBrain","parking4");
                etatMajeur = ÉtatsAutonomesMajeurs.FINIT;
                return;
            case FINIT:
                return;

        }
    }

    public void faitAllezRetour()
    {
        switch(minorState)
        {
            case ARRÊTÉ:
                // fait rien; l'état majeur  va changer l'état si il y a besoin.
                return;
            case UN_ACQUISITION:
                Log.d("AutonBrain","Current Status: itemBool: " + qObjetDansRobot + " intakeStatus " + intakeCtrlBlue.isObjectInPayload());
                if(qObjetDansRobot || intakeCtrlBlue.isObjectInPayload())
                {
                    // nous avons le bloc
                    Log.d("AutonBrain","Missed block on last run, proceeding.");
                    minorState = ÉtatsAutonomesMineurs.DEUX_MOUVEMENT_VERS_DEPOSITION;
                    return;
                }

                intakeCtrlBlue.setState(IntakeState.BASE);
                new Thread(()->{
                    boolean isInState = minorState.equals(ÉtatsAutonomesMineurs.UN_ACQUISITION);
                    Log.d("AutonBrainThread","Status0: scoop: " + qObjetDansRobot +" state " + isInState);
                    while(!qObjetDansRobot && isInState)
                    {
                        Sleep.sleep(10);
                        isInState = minorState.equals(ÉtatsAutonomesMineurs.UN_ACQUISITION);
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
                    minorState = ÉtatsAutonomesMineurs.DEUX_MOUVEMENT_VERS_DEPOSITION;
                    Log.d("AutonBrain","Continuing to deposit");
                    return;
                }
                RRctrl.followLineToSpline(positionCollectionDObjetBleu);
                Log.d("AutonBrain","Retrying to find a block");
                return;
            case DEUX_MOUVEMENT_VERS_DEPOSITION: // TODO implement go forward and then turn
                new Thread(()->{
                       while(minorState == ÉtatsAutonomesMineurs.DEUX_MOUVEMENT_VERS_DEPOSITION)
                       {
                           if(normalizedColorSensor.getNormalizedColors().alpha > Configuration.colorSensorWhiteAlpha)
                           {
                               // we know the x coordinate
                               Pose2d currentPos = RRctrl.getPos();
                               Log.d("AutonBrainThread","Current X: " + currentPos.getX());
                               RRctrl.setPos(new Pose2d(27,currentPos.getY(),currentPos.getHeading()));
                               Log.d("AutonBrainThread","New X: " + RRctrl.getPos().getX());
                           }
                       }
                       // finit l'état donc on arête le fil d'execution
                }).start();
                RRctrl.followLineToSpline(positionResetContreMurBleu);
                Pose2d currentPos = RRctrl.getPos();
                RRctrl.setPos(new Pose2d(currentPos.getX(),65.5,0));
                RRctrl.followLineToSpline(positionDépartBleu);
                RRctrl.followLineToSpline(positionDeposerBlocSurTourBleu);
                Log.d("AutonBrain","Prepare for drop off");
                minorState = ÉtatsAutonomesMineurs.TROIS_DEPOSITION;
                return;
            case TROIS_DEPOSITION:
                slideCtrl.extendDropRetract(butNormale);
                Log.d("AutonBrain","Slide drop complete");
                minorState = ÉtatsAutonomesMineurs.QUATRE_RETOUR_POUR_ACQUISITION;
                return;
            case QUATRE_RETOUR_POUR_ACQUISITION:
                RRctrl.followLineToSpline(positionContreMurAvantWarehouseBleu);
                RRctrl.setPos(new Pose2d(positionContreMurAvantWarehouseBleu.getX(),65.5,0)); // reset contre mur.
                Log.d("AutonBrain","intake prepped");
                intakeCtrlBlue.setState(IntakeState.BASE);
                RRctrl.followLineToSpline(positionCollectionDObjetBleu);
                Log.d("AutonBrain","reset status and init for intake");
                qObjetDansRobot = false; // reset
                minorState = ÉtatsAutonomesMineurs.UN_ACQUISITION;
                return;

        }
    }

    public static Pose2d positionDépartBleu = new Pose2d(14,65.5,0);
    public static Pose2d positionCollectionDObjetBleu = new Pose2d(35,65.5,0);
    public static Pose2d positionResetContreMurBleu = new Pose2d(30,75,0);
    public static Pose2d positionDeposerBlocSurTourBleu = new Pose2d(5.58,64.47,-Math.toRadians(58));
    public static Pose2d positionContreMurAvantWarehouseBleu = new Pose2d(14,80,0);
    public static Pose2d positionFinaleBleu = new Pose2d(52,80,0);


}
