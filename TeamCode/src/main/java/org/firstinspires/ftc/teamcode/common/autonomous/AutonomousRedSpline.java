package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@Autonomous(name="A - Auton (Red Spline)")
public class AutonomousRedSpline extends OpMode {
    AutonomousBrain brain;

    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.RedSpline);
    }

    @Override
    public void start() {
        brain.start();
    }

    @Override
    public void loop() {
        brain.doStateAction();
    }
}
