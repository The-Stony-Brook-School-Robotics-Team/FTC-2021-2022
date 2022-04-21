package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

import java.util.concurrent.atomic.AtomicReference;

//@Autonomous (name = "A - Auton (Blue Main)")
public class AutonomousBlueFullNew extends LinearOpMode {
    AutonomousBrain brain;
    boolean qA = false;
    boolean qContinue = false;
    AtomicReference<Boolean> masterQContinue = new AtomicReference<>();
    public static Gamepad gamepad;

    @Override
    public void runOpMode()
    {
        masterQContinue.set(true);
        OpenCVController.isDuck = false;
        brain = new AutonomousBrain(hardwareMap,telemetry, AutonomousMode.BlueStatesWarehouse);
        Log.d("Auton BF","Init Complete");
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
        brain.majorState.set(AutonomousBrain.MajorAutonomousState.FINISHED);
        brain.minorState.set(AutonomousBrain.MinorAutonomousState.STOPPED);
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
