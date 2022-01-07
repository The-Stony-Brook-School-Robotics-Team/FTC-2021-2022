package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="A - Auton (Red Full)")
public class AutonomousRedFull extends OpMode {
    AutonomousBrain brain;

    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.RedFull);
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