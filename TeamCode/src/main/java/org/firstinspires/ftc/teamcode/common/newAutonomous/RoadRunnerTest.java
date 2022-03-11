package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.enums.SlideTarget;

@Autonomous(name = "A_William - RoadRunnerTest")
public class RoadRunnerTest extends LinearOpMode {
    AutonomousClientBeta autonomousClientBeta;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClientBeta = new AutonomousClientBeta(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (true) {
                try {
                    if (autonomousClientBeta.roadRunnerDrive.isRunningFollowTrajectory)
                        Thread.sleep(10);
                    else {
                        autonomousClientBeta.roadRunnerDrive.update();
                        Thread.sleep(2);
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        localizeThread.start();

        autonomousClientBeta.readCamera();

        waitForStart();

        autonomousClientBeta.originalSlideController.extendDropRetract_NewAutonomous(SlideTarget.TOP_DEPOSIT);

        autonomousClientBeta.stopRobot();
        stop();
    }
}
