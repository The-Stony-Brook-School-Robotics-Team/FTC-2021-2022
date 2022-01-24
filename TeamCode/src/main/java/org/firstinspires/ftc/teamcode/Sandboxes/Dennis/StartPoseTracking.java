package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@Config
@TeleOp(name = "T - Pose Tracking Program", group="default")
public class StartPoseTracking extends LinearOpMode {

    private static SampleMecanumDrive drive;
    private static FtcDashboard dashboard;

    public static int startX = 0;
    public static int startY = 0;
    public static int startHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
