package org.firstinspires.ftc.teamcode.common.tuning.roadrunner;

import com.coyote.framework.core.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.RoadRunnerController;


@TeleOp(name = "T - LocalizationTest", group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RoadRunnerController drive = new RoadRunnerController(hardwareMap,telemetry);


        waitForStart();

        while (!isStopRequested()) {

            Pose2d poseEstimate = drive.getPos();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            drive.getDrive().update();
        }
    }
}
