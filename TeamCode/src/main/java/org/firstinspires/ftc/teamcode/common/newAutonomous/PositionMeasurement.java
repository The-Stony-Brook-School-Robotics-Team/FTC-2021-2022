package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A_William - PositionMeasurement")
public class PositionMeasurement extends LinearOpMode {
    AutonomousClientSafe autonomousClient;
    double startTime_s;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClientSafe(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (true) {
                autonomousClient.roadRunnerDrive.update();
                try {
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        localizeThread.start();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("X", autonomousClient.roadRunnerController.getPos().getX());
            telemetry.addData("Y", autonomousClient.roadRunnerController.getPos().getY());
            telemetry.addData("Head",Math.toDegrees(autonomousClient.roadRunnerController.getPos().getHeading()));
            telemetry.update();
        }

        localizeThread.interrupt();

        stop();
    }
}
