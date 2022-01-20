package org.firstinspires.ftc.teamcode.common.autonomous;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A - Auton (Blue Full)")
public class AutonomousBlueFull extends OpMode {
    AutonomousBrain brain;
    boolean qA = false;
    boolean qContinue = false;


    public static Gamepad gamepad;
    @Override
    public void init() {
        OpenCVController.isDuck = false;
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.BlueFull);
        Log.d("Auton BF","Init Complete");
        msStuckDetectLoop = Integer.MAX_VALUE;
        gamepad = gamepad1;
    }

    @Override
    public void start() {
        brain.launch();
    }

    @Override
    public void loop() {

        telemetry.addData("majorState", brain.majorState);
        telemetry.addData("minorState", brain.minorState);
        telemetry.addData("CameraReading", brain.heightFromDuck);
        telemetry.update();
        if(qContinue) {
            brain.doAutonAction();
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            qContinue = false;
        }
        //qContinue = true;
        if(gamepad1.a && !qA) {
            qA = true;
            qContinue = true;
            return;
        }
        else if (!gamepad1.a && qA) {
            qA = false;
        }

    }
}
