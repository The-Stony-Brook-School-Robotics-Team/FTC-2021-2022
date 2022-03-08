package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A_William - AutonomousBlue")
public class AutonomousBlue extends LinearOpMode {
    AutonomousClientD autonomousClient;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClientD(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                autonomousClient.roadRunnerDrive.update();
                try {
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

//        localizeThread.start();

        autonomousClient.readCamera();

        waitForStart();

        AutonomousTimer.startTimer();
        autonomousClient.getInitialBlockDone();

        while (opModeIsActive() && AutonomousTimer.canContinue(AutonomousTimer.CurrentState.DepositToPickUp)) {
            autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            autonomousClient.pickUp();
            autonomousClient.deposit();
        }

        autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        autonomousClient.park();

        localizeThread.interrupt();
        stop();
    }
}
