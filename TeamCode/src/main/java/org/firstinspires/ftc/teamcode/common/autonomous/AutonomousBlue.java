package org.firstinspires.ftc.teamcode.common.autonomous;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;

@Autonomous(name = "A - AutonomousBlue - William")
public class AutonomousBlue extends LinearOpMode {
    AutonomousClient autonomousClient;
    RoadRunnerController roadRunnerController;
    public static Gamepad gamepad;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueFull);
        roadRunnerController = autonomousClient.roadRunnerController;
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.
        gamepad = gamepad1;

        waitForStart();

        autonomousClient.setStartTime_s();
        autonomousClient.getInitialBlockDone();

        while (opModeIsActive() && NanoClock.system().seconds() - autonomousClient.startTime_s < 25) {
            autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            autonomousClient.goPickUpBlock();
            autonomousClient.goDeliverBlock();
        }
        autonomousClient.goParking();

        stop();
    }

}
