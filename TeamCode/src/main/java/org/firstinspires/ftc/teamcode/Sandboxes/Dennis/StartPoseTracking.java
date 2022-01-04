package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name = "T - Pose Tracking Program", group="default")
public class StartPoseTracking extends LinearOpMode {

    private static SampleMecanumDrive drive;
    private static FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while(!isStopRequested()) {
            // Dashboard
            Pose2d poseEstimate = drive.getPoseEstimate();
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, poseEstimate);

            // Telemetry
            telemetry.addData("runtime: ", getRuntime());
            telemetry.addData("x: ", poseEstimate.getX());
            telemetry.addData("y: ", poseEstimate.getY());
            telemetry.addData("heading: ", poseEstimate.getHeading());
            dashboard.sendTelemetryPacket(telemetryPacket);
            drive.update();


        }

    }
}
