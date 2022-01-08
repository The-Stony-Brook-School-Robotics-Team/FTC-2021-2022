package org.firstinspires.ftc.teamcode.common.autonomous;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "A - Auton (Blue Full)")
public class AutonomousBlueFull extends OpMode {
    AutonomousBrain brain;
    boolean qA = false;
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

        telemetry.addData("majorState", brain.majorState);
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
