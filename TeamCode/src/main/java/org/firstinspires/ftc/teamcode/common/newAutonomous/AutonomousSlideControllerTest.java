package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A - AutonomousAlideControllerTest - William")
public class AutonomousSlideControllerTest extends LinearOpMode {
    AutonomousClient autonomousClient;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);

        waitForStart();

        autonomousClient.slideController.setHeightTo_Autonomous(AutonomousSlideController.vertServoPosition_THREE_DEPOSIT);
        Thread.sleep(1000);
        autonomousClient.slideController.setHeightTo_Autonomous(AutonomousSlideController.vertServoPosition_PARKED);

        stop();
    }
}
