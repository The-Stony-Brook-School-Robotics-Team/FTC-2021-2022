package org.firstinspires.ftc.teamcode.common.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "A - Auton (Blue Full)")
public class AutonomousBlueFull extends OpMode {
    AutonomousBrain brain;

    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.BlueFull);
        Log.d("Auton BF","Init Complete");
    }

    @Override
    public void start() {
        brain.launch();
    }

    @Override
    public void loop() {
        brain.doAutonAction();
    }
}
