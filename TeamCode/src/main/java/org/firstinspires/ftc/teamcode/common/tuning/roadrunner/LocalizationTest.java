package org.firstinspires.ftc.teamcode.common.tuning.roadrunner;

import static org.firstinspires.ftc.teamcode.drive.OdometryLocalizer.encoderTicksToInches;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;


public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerController drive = new RoadRunnerController(hardwareMap,telemetry);
        ExpansionHubEx ctrlhub = hardwareMap.get(ExpansionHubEx.class,"Control Hub");


        waitForStart();

        while (!isStopRequested()) {
            drive.getDrive().setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            -gamepad1.right_stick_x
                    )
            );

            Pose2d poseEstimate = drive.getPos();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            drive.getDrive().update();
            RevBulkData bulk = ctrlhub.getBulkInputData();
            telemetry.addData("L",encoderTicksToInches(-bulk.getMotorCurrentPosition(1)));
            telemetry.addData("R",encoderTicksToInches(bulk.getMotorCurrentPosition(2)));
            telemetry.addData("C",encoderTicksToInches(-bulk.getMotorCurrentPosition(0)));
            telemetry.update();
        }
    }
}
