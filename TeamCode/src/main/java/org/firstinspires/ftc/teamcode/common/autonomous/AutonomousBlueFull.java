package org.firstinspires.ftc.teamcode.common.autonomous;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous (name = "A - Auton (Blue Full)")
public class AutonomousBlueFull extends LinearOpMode {
    AutonomousBrain brain;
    boolean qA = false;
    boolean qContinue = false;
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

        brain.launch();

        autonBrainExecutor.start();
        while(opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("majorState", brain.majorState);
            telemetry.addData("minorState", brain.minorState);
            telemetry.addData("CameraReading", brain.heightFromDuck);
            telemetry.addData("DepositHeight", brain.targetCarousel);
            telemetry.update();
        }
        // stop requested
        autonBrainExecutor.interrupt();
        brain.majorState = AutonomousBrain.AutonomousStates.FINISHED;
        brain.minorState = AutonomousBrain.AutonomousBackForthSubStates.STOPPED;
    }


    Thread autonBrainExecutor = new Thread(()->{
        while(opModeIsActive()&& !isStopRequested()){
                if(qContinue) {
                    brain.doAutonAction();
                    sleep(100);
                    qContinue = false;
                }
                qContinue = true;
                if(gamepad1.a && !qA) {
                    qA = true;
                    qContinue = true;
                    return;
                }
                else if (!gamepad1.a && qA) {
                    qA = false;
                }
                if(brain.majorState.equals(AutonomousBrain.AutonomousStates.FINISHED))
                {
                    requestOpModeStop();
                }
        }
    });

}
