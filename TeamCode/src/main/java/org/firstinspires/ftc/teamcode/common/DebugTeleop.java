package org.firstinspires.ftc.teamcode.common;

import com.coyote.framework.core.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="T - Robot Debugger")
public class DebugTeleop extends LinearOpMode {

    private Servo scooperBlue;
    private Servo scooperRed;
    private Servo selectedServo = scooperBlue;

    // Drive
    SampleMecanumDrive drive;

    boolean pX, pB, pY;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Roadrunner
        drive = new SampleMecanumDrive(hardwareMap);

        scooperBlue = hardwareMap.get(Servo.class, "bi");
        scooperRed = hardwareMap.get(Servo.class, "ri");

        // Up
        if(gamepad1.a && !pX) {
            pX = true;
        } else if(!gamepad1.a && pX) {
            selectedServo.setPosition(selectedServo.getPosition() + 0.025);
            pX = false;
        }

        // Down
        if(gamepad1.y && !pB) {
            pB = true;
        } else if(!gamepad1.y && pB) {
            selectedServo.setPosition(selectedServo.getPosition() - 0.025);
            pB = false;
        }

        // Switch Sides
        if(gamepad1.a && !pY) {
            pY = true;
        } else if(!gamepad1.a && pY) {
            if(selectedServo == scooperBlue) {
                selectedServo = scooperRed;
            } else if(selectedServo == scooperRed) {
                selectedServo = scooperBlue;
            }
            pY = false;
        }


        // Wait For Start
        waitForStart();


        while(!isStopRequested()) {
            telemetry.addData("Blue Servo Pos: ", scooperBlue.getPosition());
            telemetry.addData("Red Servo Pos: ", scooperRed.getPosition());







            // Drive The Robot
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            telemetry.update();
        }

    }
}
