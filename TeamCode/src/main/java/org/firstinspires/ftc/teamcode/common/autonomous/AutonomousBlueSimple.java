package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@Autonomous(name="A - Auton (Blue Simple)")
public class AutonomousBlueSimple extends OpMode {
    AutonomousBrain brain;

    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.BlueSimple);
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
