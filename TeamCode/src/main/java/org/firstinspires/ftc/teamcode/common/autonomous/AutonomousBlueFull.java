package org.firstinspires.ftc.teamcode.common.autonomous;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous (name = "A - Auton (Blue Full)")
public class AutonomousBlueFull extends LinearOpMode {
    AutonomousBrain brain;
    boolean qA = false;
    boolean qContinue = false;
    boolean masterQContinue = true;
    public static Gamepad gamepad;

    @Override
    public void runOpMode()
    {
        OpenCVController.isDuck = false;
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.BlueFull);
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
        masterQContinue = false; // master switch
        autonBrainExecutor.interrupt();
        try {
            autonBrainExecutor.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        brain.majorState = AutonomousBrain.MajorAutonomousState.FINISHED;
        brain.minorState = AutonomousBrain.MinorAutonomousState.STOPPED;
        requestOpModeStop();
        stop();
    }


    Thread autonBrainExecutor = new Thread(()->{
        while(opModeIsActive()&& !isStopRequested()){
            if(!masterQContinue) {break;}
            brain.doStateAction();
            if(brain.majorState.equals(AutonomousBrain.MajorAutonomousState.FINISHED)) { requestOpModeStop(); }
            if(!masterQContinue) { break; }

        }
    });

}
