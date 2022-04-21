package org.firstinspires.ftc.teamcode.common.tankAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.sbs.bears.Tank.NewSlideController;
import org.sbs.bears.Tank.SlideConstants;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "A - Auton (Red Tank Deposit WAREHOUSE)")
public class AutonomousRedDepositWarehouse extends LinearOpMode {
    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Init in progress...");
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        NewSlideController slide = new NewSlideController(hardwareMap);
        driver.setPos(startPosition);
        telemetry.addData("Status","Init complete");
        telemetry.addData("Status","Init complete");
        waitForStart();
        telemetry.addData("Status","Slide init");
        telemetry.addData("Status","Traj init");
        AtomicReference<Boolean> terminator = new AtomicReference<>();
        terminator.set(false);
        // STEP 1: We start in front of the hub, so deposit.
        slide.extendDropRetract(SlideConstants.slideMotorPosition_THREE_CLOSE,SlideConstants.flipper_THREE_CLOSE,SlideConstants.potentiometer_THREE_DEPOSIT);
        telemetry.addData("Status","Slide retracting...");
        Sleep.sleep(1000);
        telemetry.addData("Status","traj in progress...");
        // STEP 2: Park in Warehouse
        driver.goBackwardGyro(45, 0.5,terminator,P,I,D);
        telemetry.addData("Status","traj done");
    }
    Pose2d startPosition = new Pose2d(-12,-65.5,0);

}
