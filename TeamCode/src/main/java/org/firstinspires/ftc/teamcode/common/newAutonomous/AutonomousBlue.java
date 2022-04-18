package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

//@Autonomous(name = "----- Autonomous Blue By William Lord Tao")
public class AutonomousBlue extends LinearOpMode {
    AutonomousClient autonomousClient;
    private int counter = 0;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClient(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (true) {
                try {
                    if (!opModeIsActive()) {
                        autonomousClient.stopRoadRunner();
                        autonomousClient.needToStopAllThreads = true;
                        return;
                    }
                    if (autonomousClient.roadRunnerDrive.isRunningFollowTrajectory)
                        Thread.sleep(100);
                    else {
                        autonomousClient.roadRunnerDrive.update();
                        Thread.sleep(10);
                    }
                } catch (InterruptedException e) {
                    autonomousClient.stopRoadRunner();
                    e.printStackTrace();
                }
            }
        });

        waitForStart();

        localizeThread.start();
        AutonomousTimer.startTimer();

        autonomousClient.readCamera();
        autonomousClient.getInitialBlockDone();
        counter++;

        while (opModeIsActive() && AutonomousTimer.canContinue(AutonomousTimer.CurrentState.DepositToPickUp)) {
            autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            autonomousClient.pickUp();
            autonomousClient.deposit();
            counter++;
        }

        if (!AutonomousTimer.canContinue()) {
            autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
            //telemetry.addData("Result: ", counter + " + 1");
            //telemetry.update();
            autonomousClient.park();
        }

        autonomousClient.needToStopAllThreads = true;
        localizeThread.interrupt();
        autonomousClient.stopRoadRunner();
        stop();
    }
}
