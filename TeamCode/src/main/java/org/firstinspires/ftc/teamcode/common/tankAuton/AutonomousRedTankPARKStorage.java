package org.firstinspires.ftc.teamcode.common.tankAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewBlueIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewRedIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewSlideController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.SlideConstants;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.enums.IntakeState;

import java.util.concurrent.atomic.AtomicReference;

@Autonomous (name = "A - Auton (Red Tank PARK STORAGE)")
public class AutonomousRedTankPARKStorage extends LinearOpMode {

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
        driver.goForwardGyro(45, 0.5,terminator,P,I,D);
        telemetry.addData("Auton","Traj Done...");
        telemetry.update();
    }






}
