package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

//@Autonomous(name = "A_William - AutonomousTrajectoryTest")
public class AutonomousTrajectoryTest extends LinearOpMode {
    AutonomousClientSafe autonomousClientBeta;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClientBeta = new AutonomousClientSafe(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);

        waitForStart();

//        autonomousClient.getInitialBlockDone();
        autonomousClientBeta.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        autonomousClientBeta.pickUp();
//        autonomousClient.goDeliverBlock();
//        autonomousClient.goParking();

        stop();
    }
}
