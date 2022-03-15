package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.sbs.bears.robotframework.controllers.OpenCVController;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous (name = "A - Auton (Red Duck Warehouse)")
public class AutonomousRedSimpleWarehouse extends LinearOpMode {
    AutonomousBrainSimple brain;
    boolean qA = false;
    boolean qContinue = false;
    AtomicReference<Boolean> masterQContinue = new AtomicReference<>();
    public static Gamepad gamepad;

    @Override
    public void runOpMode()
    {
        masterQContinue.set(true);
        OpenCVController.isDuck = false;
        brain = new AutonomousBrainSimple(hardwareMap,telemetry,AutonomousMode.RedStatesDuckSimpleWarehouse);
        Log.d("Auton BS","Init Complete");
        msStuckDetectLoop = Integer.MAX_VALUE;
        gamepad = gamepad1;

        waitForStart();

        brain.start();

        autonBrainExecutor.start();
        while(opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("majorState", brain.majorState);
            telemetry.addData("minorState", brain.minorState);
            telemetry.addData("CameraReading", brain.heightFromDuck);
            telemetry.addData("DepositHeight", brain.iniTarget);
            telemetry.addData("isParkingAvailable", brain.isParkingAvailable);
            telemetry.update();
        }
        // stop requested
        masterQContinue.set(false);
        autonBrainExecutor.interrupt();
        try {
            autonBrainExecutor.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        brain.majorState.set(AutonomousBrainSimple.MajorAutonomousState.FINISHED);
        brain.minorState.set(AutonomousBrainSimple.MinorAutonomousState.STOPPED);
        requestOpModeStop();
        stop();
    }


    Thread autonBrainExecutor = new Thread(()->{
        while(opModeIsActive() && !isStopRequested()){
            if(!masterQContinue.get()) {break;}
            brain.doStateAction();
            if(brain.majorState.get().equals(AutonomousBrain.MajorAutonomousState.FINISHED)) { requestOpModeStop(); }
            if(!masterQContinue.get()) { break; }

        }
    });

}
