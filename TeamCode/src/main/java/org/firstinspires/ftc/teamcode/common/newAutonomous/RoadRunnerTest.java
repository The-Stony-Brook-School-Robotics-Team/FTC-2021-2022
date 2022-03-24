package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A_William - RoadRunnerTest")
public class RoadRunnerTest extends LinearOpMode {
    AutonomousClient autonomousClient;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (true) {
                try {
                    if (autonomousClient.roadRunnerDrive.isRunningFollowTrajectory)
                        Thread.sleep(100);
                    else {
                        autonomousClient.roadRunnerDrive.update();
                        Thread.sleep(2);
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

//        localizeThread.start();

//        autonomousClientBeta.readCamera();

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            autonomousClient.stopRoadRunner();
        }).start();

        while (opModeIsActive()) {
            autonomousClient.runRawPickUpTrajectory();
            autonomousClient.runRawDepositTrajectory();
        }

        stop();
    }
}
