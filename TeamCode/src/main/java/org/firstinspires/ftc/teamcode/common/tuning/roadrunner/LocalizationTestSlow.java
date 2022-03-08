package org.firstinspires.ftc.teamcode.common.tuning.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
@Disabled
@Config
public class LocalizationTestSlow extends LinearOpMode {
    public static double factor = 0.3;
    private boolean qUP;
    private boolean qDOWN;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            -factor*gamepad1.left_stick_y,
                            -factor*gamepad1.left_stick_x,
                            -factor*gamepad1.right_stick_x
                    )
            );*/
            if(gamepad1.dpad_up) {
                drive.setWeightedDrivePower(new Pose2d(
                        1,0,-gamepad1.right_stick_x
                ));
            }
           else  if(gamepad1.dpad_right) {
                drive.setWeightedDrivePower(new Pose2d(
                        0,-1,-gamepad1.right_stick_x
                ));
            }
            else  if(gamepad1.dpad_left) {
                drive.setWeightedDrivePower(new Pose2d(
                        0,1,-gamepad1.right_stick_x
                ));
            }
            else  if(gamepad1.dpad_down) {
                drive.setWeightedDrivePower(new Pose2d(
                        -1,0,-gamepad1.right_stick_x
                ));
            }
            else {
                drive.setWeightedDrivePower(new Pose2d(
                        0,0,-gamepad1.right_stick_x
                ));
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Factor",factor);

            telemetry.update();

        }
    }
}
