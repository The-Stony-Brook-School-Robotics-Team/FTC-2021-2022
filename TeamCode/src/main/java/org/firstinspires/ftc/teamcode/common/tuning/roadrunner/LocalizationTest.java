package org.firstinspires.ftc.teamcode.common.tuning.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "T - LocalizationTest", group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0.1*gamepad1.left_stick_y,
                            -0.1*gamepad1.left_stick_x,
                            -0.1*gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX()*1.066*1.036*0.952);
            telemetry.addData("y", poseEstimate.getY()*1.066*0.91*1.179);
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
