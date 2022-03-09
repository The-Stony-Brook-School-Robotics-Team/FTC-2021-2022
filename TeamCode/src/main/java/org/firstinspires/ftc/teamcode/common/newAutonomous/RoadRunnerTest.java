package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
                        Thread.sleep(10);
                    else {
                        autonomousClient.roadRunnerDrive.update();
                        Thread.sleep(2);
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        localizeThread.start();

        autonomousClient.readCamera();

        waitForStart();

        autonomousClient.roadRunnerDrive.followTrajectory(
                autonomousClient.roadRunnerDrive.trajectoryBuilder(autonomousClient.roadRunnerDrive.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(45.0, 64.5, Math.toRadians(40.0)), Math.toRadians(40.0))
                        .build()
        );

        localizeThread.interrupt();
        autonomousClient.stopRobot();
        stop();
    }
}
