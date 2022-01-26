package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//@Autonomous(name="A - Auton (Red Simple)")
public class AutonomousRedSimple extends OpMode {
    AutonomousBrain brain;

    @Override
    public void init() {
        brain = new AutonomousBrain(hardwareMap,telemetry,AutonomousMode.RedSimple);
    }

    @Override
    public void start() {
        brain.lance();
    }

    @Override
    public void loop() {
        brain.faitActionAutonome();
    }
}
