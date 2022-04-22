package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.sbs.bears.robotframework.controllers.DrivingControllerTank;

@Autonomous(name = "A - debugger")
public class driving extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        waitForStart();
        driver.turnRSpecial(90,0.3);
    }
}
