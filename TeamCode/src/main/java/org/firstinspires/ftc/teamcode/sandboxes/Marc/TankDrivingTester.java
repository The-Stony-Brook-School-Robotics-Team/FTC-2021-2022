package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstantsTank;
import org.sbs.bears.robotframework.controllers.DrivingControllerTank;
@TeleOp(name = "A - Tank Driving Tester")
public class TankDrivingTester extends LinearOpMode {
    private boolean qB;

    @Override
    public void runOpMode() throws InterruptedException {
        DrivingControllerTank driver = new DrivingControllerTank(hardwareMap);
        waitForStart();
        boolean qA = false;
        while(opModeIsActive() && !isStopRequested())
        {
            driver.RR.setMotorPowers(-gamepad1.left_stick_x +gamepad1.right_stick_x, -gamepad1.left_stick_x - gamepad1.right_stick_x);
            if(gamepad1.a && !qA)
            {
                qA = true;
                driver.stopMotors();
                driver.goForwardSimple(10,0.2);
            }
            if(!gamepad1.a && qA)
            {
                qA = false;
            }
            if(gamepad1.b && !qB)
            {
                qB = true;
                driver.stopMotors();
                driver.goForward(10,2);
            }
            if(!gamepad1.b && qB)
            {
                qB = false;
            }
        }
    }
}
