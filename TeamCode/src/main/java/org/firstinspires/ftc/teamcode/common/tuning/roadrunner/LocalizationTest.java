package org.firstinspires.ftc.teamcode.common.tuning.roadrunner;

import static org.firstinspires.ftc.teamcode.drive.OdometryLocalizer.encoderTicksToInches;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;

@TeleOp
public class LocalizationTest extends LinearOpMode {
    Encoder leftEncoder,rightEncoder,frontEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerController drive = new RoadRunnerController(hardwareMap,telemetry);
        ExpansionHubEx ctrlhub = hardwareMap.get(ExpansionHubEx.class,"Control Hub");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftodom"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightodom"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "duck"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
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
            telemetry.addData("L",encoderTicksToInches(leftEncoder.getCurrentPosition()));
            telemetry.addData("R",encoderTicksToInches(rightEncoder.getCurrentPosition()));
            telemetry.addData("C",encoderTicksToInches(frontEncoder.getCurrentPosition()));
            telemetry.update();
        }
    }
}
