package org.firstinspires.ftc.teamcode.common.tankAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.sbs.bears.robotframework.controllers.DrivingControllerTank;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous (name = "A - Auton (Blue Tank PARK STORAGE)")
public class AutonomousBlueTankPARKStorage extends LinearOpMode {

    public static double P = 2;
    public static double I = 0;
    public static double D = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Auton","Init Progress...");
        telemetry.update();
        msStuckDetectStart = Integer.MAX_VALUE;
        msStuckDetectInit = Integer.MAX_VALUE;
        msStuckDetectLoop = Integer.MAX_VALUE;
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        telemetry.addData("Auton","Init Complete");
        telemetry.update();


        waitForStart();


        AtomicReference<Boolean> terminator = new AtomicReference<>();
        telemetry.addData("Auton","Traj Progres...");
        telemetry.update();
        driver.goBackwardGyro(45, 0.5,terminator,P,I,D);
        telemetry.addData("Auton","Traj Done...");
        telemetry.update();
    }





}
