package org.firstinspires.ftc.teamcode.common.tankAuton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.Tank.NewSlideController;
import org.sbs.bears.Tank.SlideConstants;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous(name = "A - Auton (Red Tank Cam Deposit WAREHOUSE)")
public class AutonomousRedDepositCamWarehouse extends LinearOpMode {
    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    public static int slideExt1 = 600;
    public static int slideExt2 = 600;
    public static int slideExt3 = 600;
    public static double flipper1 = 0.05;
    public static double flipper2 = 0.2;
    public static double flipper3 = 0.4;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Init in progress...");
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        NewSlideController slide = new NewSlideController(hardwareMap);
        OpenCVController CV = new OpenCVController(hardwareMap,telemetry, AutonomousMode.RedTankFULL);
        OpenCVController.doAnalysisMaster = true;
        driver.setPos(startPosition);
        telemetry.addData("Status","Init complete");
        waitForStart();
        TowerHeightFromDuck height = CV.getWhichTowerHeight();
        telemetry.addData("Status","Slide init");
        AtomicReference<Boolean> terminator = new AtomicReference<>();
        terminator.set(false);
        // STEP 1: We start in front of the hub, so deposit.
        int slideExt = slideExt3;
        double flipper = flipper3;
        switch(height)
        {
            case ONE:
                slideExt = slideExt1;
                flipper = flipper1;
                break;
            case TWO:
                slideExt = slideExt2;
                flipper = flipper2;
                break;
            case THREE:
                slideExt = slideExt3;
                flipper = flipper3;
                break;
        }
        slide.extendDropRetract(slideExt,flipper,SlideConstants.potentiometer_THREE_DEPOSIT);
        telemetry.addData("Status","Slide retracting...");
        telemetry.update();
        Sleep.sleep(1000);
        telemetry.addData("Status","traj in progress...");
        telemetry.update();
        // STEP 2: Park in Warehouse
        driver.goBackwardGyro(48, 0.5,terminator,P,I,D);
        telemetry.addData("Status","traj done");
        telemetry.update();
        telemetry.addData("Status","traj2 done");
        telemetry.update();
        telemetry.addData("Status","turn2 in progress...");
        telemetry.update();

    }
    Pose2d startPosition = new Pose2d(-12,65.5,0);

}
