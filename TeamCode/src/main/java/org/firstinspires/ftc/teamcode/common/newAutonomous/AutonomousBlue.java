package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A - AutonomousBlue - William")
public class AutonomousBlue extends LinearOpMode {
    AutonomousClient autonomousClient;
    double startTime_s;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueFull);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        waitForStart();

        startTime_s = NanoClock.system().seconds();
        autonomousClient.getInitialBlockDone();

        while (opModeIsActive() && NanoClock.system().seconds() - startTime_s < 25) {
            autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            autonomousClient.goPickUpBlock();
            autonomousClient.goDepositBlock();
        }

        autonomousClient.goParking();
        stop();
    }
}
