package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "A - Auton (Blue Full)")
public class AutonomousBlueFull extends OpMode {
    AutonomousBrain brain;
    boolean qContinue = false;
    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.BlueFull);
        Log.d("Auton BF","Init Complete");
        msStuckDetectLoop = Integer.MAX_VALUE;
    }

    @Override
    public void start() {
        brain.launch();
    }

    @Override
    public void loop() {
        if(qContinue) {
            brain.doAutonAction();
        }
        if(!gamepad1.a) {
            qContinue = false;
            return;
        }
        qContinue = true;
    }
}
