package org.firstinspires.ftc.teamcode.common.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//@Autonomous(name = "A - Auton (Blue Spline)")
public class AutonomousBlueSpline extends OpMode {
    AutonomousBrain brain;

    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.BlueSpline);
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
