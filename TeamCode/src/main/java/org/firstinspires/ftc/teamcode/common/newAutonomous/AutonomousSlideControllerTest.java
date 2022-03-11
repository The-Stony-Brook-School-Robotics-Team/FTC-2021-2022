package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A_William - AutonomousSlideControllerTest")
public class AutonomousSlideControllerTest extends LinearOpMode {
    AutonomousClientBeta autonomousClientBeta;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVController.isDuck = false;
        autonomousClientBeta = new AutonomousClientBeta(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);

        waitForStart();

        autonomousClientBeta.slideController.setHeightTo_Autonomous(AutonomousSlideController.vertServoPosition_THREE_DEPOSIT);
        Thread.sleep(1000);
        autonomousClientBeta.slideController.setHeightTo_Autonomous(AutonomousSlideController.vertServoPosition_PARKED);

        stop();
    }
}
