package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

//@Autonomous(name = "A_William - AutonomousTrajectoryTest")
public class AutonomousTrajectoryTest extends LinearOpMode {
    AutonomousClient autonomousClient;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);

        waitForStart();

        Thread stopTrajectoryThread = new Thread(() -> {
            while (opModeIsActive()) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            autonomousClient.stopRoadRunner();
        });

        stopTrajectoryThread.start();

        autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        while (opModeIsActive()) {
            autonomousClient.runRawPickUpTrajectory();
            autonomousClient.runRawDepositTrajectory();
        }

        try {
            stopTrajectoryThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        stop();
    }
}
