package org.firstinspires.ftc.teamcode.common.tankAuton;

import android.util.Log;

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

@Autonomous(name = "A - Auton (Red Tank Deposit2 WAREHOUSE)")
public class AutonomousRedDepositWarehouse2 extends LinearOpMode {
    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    NewSlideController slide;

    public static int slideDistance3Auton = 500;
    public static int slideDistance2Auton = 500;
    public static int slideDistance1Auton = 500;
    public static double flipper3Auton = .32;
    public static double flipper2Auton = 0.12;
    public static double flipper1Auton = 0.05;

    public static double targetPot = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Init in progress...");
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
         slide = new NewSlideController(hardwareMap);
        OpenCVController CV = new OpenCVController(hardwareMap,telemetry, AutonomousMode.BlueTankFULL);
        driver.setPos(startPosition);
        double iniPot = slide.potentiometer.getVoltage();
        Log.d("Auton","ini pot: " + iniPot + " target: " + targetPot);
        setHeight(targetPot);
        OpenCVController.doAnalysisMaster = true;
        CV.getWhichTowerHeight();
        telemetry.addData("Status","Init complete");


        waitForStart();


        TowerHeightFromDuck duck = CV.getWhichTowerHeight();
        OpenCVController.doAnalysisMaster = false;
        telemetry.addData("Status","Slide init");
        AtomicReference<Boolean> terminator = new AtomicReference<>();
        terminator.set(false);
        // STEP 1: We start in front of the hub, so deposit.
        int slideDistance = 0;
        double flipperAmount = 0;
        switch(duck)
        {
            case ONE:
                slideDistance = slideDistance1Auton;
                flipperAmount = flipper1Auton;
                break;
            case TWO:
                slideDistance = slideDistance2Auton;
                flipperAmount = flipper2Auton;
                break;
            case THREE:
                slideDistance = slideDistance3Auton;
                flipperAmount = flipper3Auton;
                break;
        }
        slide.extendDropRetract(slideDistance,flipperAmount,0);
        telemetry.addData("Status","Slide retracting...");
        Sleep.sleep(1000);
        telemetry.addData("Status","traj in progress...");
        // STEP 2: Park in Warehouse
        driver.goBackwardGyro(48, 0.5,terminator,P,I,D);
        telemetry.addData("Status","traj done");
        telemetry.addData("Status","turn in progress...");
        // STEP 2: Park in Warehouse
        driver.turnLSpecial(45,0.3);
        telemetry.addData("Status","turn done");
        telemetry.addData("Status","traj2 in progress...");
        // STEP 2: Park in Warehouse
        driver.goBackwardGyro(24, 0.5,terminator,P,I,D);
        telemetry.addData("Status","traj2 done");
        telemetry.addData("Status","turn2 in progress...");
        // STEP 2: Park in Warehouse
        driver.turnR(45,0.3);
        telemetry.addData("Status","turn2 done");
        telemetry.addData("Status","traj3 in progress...");
        // STEP 2: Park in Warehouse
        driver.goBackwardGyro(24, 0.5,terminator,P,I,D);
        telemetry.addData("Status","traj3 done");
        telemetry.addData("Status","turn3 in progress...");
        driver.turnLSpecial(45,0.3);
        telemetry.addData("Status","turn3 done");
        telemetry.addData("Status","done complete");
        Thread.sleep(1000);


    }
    Pose2d startPosition = new Pose2d(-12,65.5,0);


    public void setHeight(double desiredVoltage)
    {
        if(slide.potentiometer.getVoltage() < desiredVoltage) {
            while (slide.potentiometer.getVoltage() < desiredVoltage) {
                slide.liftMotor.setPower(SlideConstants.liftMotorPower_MOVING);
            }
            slide.liftMotor.setPower(0);
            return;
        }
        while (slide.potentiometer.getVoltage() > desiredVoltage){
            try {
                slide.liftMotor.setPower(-SlideConstants.liftMotorPower_MOVING);
            }
            catch (Exception e)
            {
                e.printStackTrace();
                Log.d("NewSlideController","Slide height failed");
            }
        }
        try {
            slide.liftMotor.setPower(0);
        }
        catch(Exception e)
        {
            e.printStackTrace();
            Log.d("NewSlideController","Slide2 height failed");

        }
    }
}
