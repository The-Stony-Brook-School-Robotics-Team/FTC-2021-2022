package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.OpenCVController;

@Autonomous(name = "A - Auton (Blue Main William)")
public class AutonomousBlue extends LinearOpMode {
    AutonomousClientSafe autonomousClient;
    private int counter = 0;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClient = new AutonomousClientSafe(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (true) {
                try {
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

        localizeThread.start();

        autonomousClient.readCamera();

        waitForStart();

        AutonomousTimer.startTimer();
        autonomousClient.getInitialBlockDone();

        while (opModeIsActive() && AutonomousTimer.canContinue(AutonomousTimer.CurrentState.DepositToPickUp)) {
            autonomousClient.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            autonomousClient.pickUp();
            autonomousClient.deposit();
           // counter++;
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
