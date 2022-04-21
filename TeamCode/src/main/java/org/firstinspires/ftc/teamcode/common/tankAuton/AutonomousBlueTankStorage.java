package org.firstinspires.ftc.teamcode.common.tankAuton;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import android.graphics.Path;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewBlueIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewRedIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewSlideController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.SlideConstants;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous (name = "A - Auton (Blue Tank STORAGE)")
public class AutonomousBlueTankStorage extends LinearOpMode {

    public static double P = 1;
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
        bu.setState(IntakeState.PARK);
        OpenCVController.isDuck = false;
        doAnalysisMaster = true;
        driver.setPos(startPosition);
        waitForStart();

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

        // STEP 4: Park
        driver.goBackwardGyro(distance, 0.5,terminator,P,I,D);
        driver.turnL(45,0.2);
        driver.goBackwardGyro(44, 0.5,terminator,P,I,D);

    }

    Pose2d startPosition = new Pose2d(-12,65.5,0);
    Pose2d depositPosition = new Pose2d(-31.5,65.5,0);



}
