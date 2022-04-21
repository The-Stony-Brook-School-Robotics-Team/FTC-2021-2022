package org.firstinspires.ftc.teamcode.common.tankAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewBlueIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewRedIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewSlideController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.SlideConstants;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.enums.IntakeState;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous (name = "A - Auton (Red Tank PARK STORAGE)")
public class AutonomousRedTankPARKStorage extends LinearOpMode {

    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    @Override
    public void runOpMode() {
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        OpenCVController CV = new OpenCVController(hardwareMap,telemetry,AutonomousMode.BlueTankFULL);
        NewSlideController slide = new NewSlideController(hardwareMap);
        NewBlueIntakeController bu = new NewBlueIntakeController(hardwareMap,slide.getClaw(),slide.getDistanceSensor());
        NewRedIntakeController red = new NewRedIntakeController(hardwareMap,slide.getClaw(),slide.getDistanceSensor());
        slide.setTargetHeight(SlideConstants.potentiometer_THREE_DEPOSIT);
        red.setState(IntakeState.PARK);
        bu.setState(IntakeState.PARK);/*
        OpenCVController.isDuck = false;
        doAnalysisMaster = true;*/
        driver.setPos(startPosition);
        waitForStart();
        long iniTime = System.nanoTime();
/*
        // STEP 1: Camera
        TowerHeightFromDuck heightFromDuck = CV.getWhichTowerHeight();
        Log.d("height: ", heightFromDuck.toString());

        // STEP 2: Move To Ini Deposit
        double distance = RoadRunnerController.distanceTwoPoints(startPosition,depositPosition);
        AtomicReference<Boolean> terminator = new AtomicReference<>();
        terminator.set(false);
        driver.goForwardGyro(distance,0.5,terminator,P,I,D);
        //driver.goForwardSimple(distance,0.5);

        // STEP 3: Deposit
        slide.extendDropRetract(SlideConstants.slideMotorPosition_THREE_CLOSE,SlideConstants.flipper_THREE_CLOSE,SlideConstants.potentiometer_THREE_DEPOSIT);
*/
        // STEP 4: Park
        AtomicReference<Boolean> terminator = new AtomicReference<>();
        while((System.nanoTime() - iniTime) < 25*1E9)
        {
            Sleep.sleep(10);
        }
        //double distance = RoadRunnerController.distanceTwoPoints(driver.getPos(),finalPosition);
        driver.goForwardGyro(45, 1,terminator,P,I,D);
        requestOpModeStop();
    }

    Pose2d startPosition = new Pose2d(-12,-65.5,0);




}
