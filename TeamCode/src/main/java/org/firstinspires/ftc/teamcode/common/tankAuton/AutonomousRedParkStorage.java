package org.firstinspires.ftc.teamcode.common.tankAuton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "A - Auton (Red Tank PARK STORAGE)")
public class AutonomousRedParkStorage extends LinearOpMode {

    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status","Init in progress...");
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        driver.setPos(startPosition);
        telemetry.addData("Status","Init complete");
        waitForStart();
        telemetry.addData("Status","Traj init");
        AtomicReference<Boolean> terminator = new AtomicReference<>();
        terminator.set(false);
        telemetry.addData("Status","Traj in progress...");
        Thread traj = new Thread(()->{
            driver.goForwardGyro(40, 0.5,terminator,P,I,D);
        });
        traj.start();
        long iniTime = System.nanoTime();
        long delta = 0;
        while(traj.isAlive() && (delta < 10))
        {
            delta = System.nanoTime() - iniTime;
            Sleep.sleep(10);
        }
        if(traj.isAlive())
        {
            traj.interrupt();
            driver.stopMotors();
        }
        telemetry.addData("Status","Traj complete");
    }

    Pose2d startPosition = new Pose2d(-12,65.5,0);
}
