package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A - AutonomousTrajectoryTest - William")
public class AutonomousTrajectoryTest extends LinearOpMode {
    AutonomousClient autonomousClient;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueFull);

        waitForStart();

//        autonomousClient.getInitialBlockDone();
        autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        autonomousClient.goPickUpBlock();
//        autonomousClient.goParking();

        stop();
    }
}
