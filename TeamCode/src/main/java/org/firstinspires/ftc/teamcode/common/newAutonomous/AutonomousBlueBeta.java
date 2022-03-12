package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.controllers.OpenCVController;

//@Autonomous(name = "A_William - AutonomousBlue - BETA")
public class AutonomousBlueBeta extends LinearOpMode {
    AutonomousClientBeta autonomousClientBeta;

    @Override
    public void runOpMode() {
        OpenCVController.isDuck = false;
        autonomousClientBeta = new AutonomousClientBeta(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        msStuckDetectLoop = Integer.MAX_VALUE;  //Turn off infinite loop detection.

        Thread localizeThread = new Thread(() -> {
            while (true) {
                try {
                    autonomousClientBeta.roadRunnerDrive.update();
                    Thread.sleep(2);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

//        localizeThread.start();

        autonomousClientBeta.readCamera();

        waitForStart();

        AutonomousTimer.startTimer();
        autonomousClientBeta.getInitialBlockDone();

        while (opModeIsActive() && AutonomousTimer.canContinue(AutonomousTimer.CurrentState.DepositToPickUp)) {
            autonomousClientBeta.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            autonomousClientBeta.pickUp();
            autonomousClientBeta.deposit();
        }

        autonomousClientBeta.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        autonomousClientBeta.park();

        localizeThread.interrupt();
        autonomousClientBeta.stopRobot();
        stop();
    }
}
