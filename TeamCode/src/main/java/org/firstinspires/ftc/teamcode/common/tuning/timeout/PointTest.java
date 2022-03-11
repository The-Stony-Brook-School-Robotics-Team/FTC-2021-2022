package org.firstinspires.ftc.teamcode.common.tuning.timeout;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.timeout.CustomTimeoutTuningDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

//@Autonomous(group = "drive", name="**INFINITE** PointTest")
public class PointTest extends LinearOpMode {

    // Config
    public static double DISTANCE = 48;

    // Internal
    private enum states { STILL, READY, WORKING }
    private static states currentState = states.STILL;
    // Buttons
    private static boolean pA = false;
    // Dashboard
    private static FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        CustomTimeoutTuningDrive drive = new CustomTimeoutTuningDrive(hardwareMap,Double.MAX_VALUE);
        drive.setPoseEstimate(new Pose2d(0, 0));

        currentState = states.READY;
        telemetry.addData("Status: ", currentState);
        telemetry.update();

        // On Start
        waitForStart();

        telemetry.clearAll();

        // Work Loop
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.a && !pA) {
                pA = true;
            } else if(!gamepad1.a && pA) {
                switch(currentState) {
                    case WORKING:
                    case READY:
                        currentState = states.STILL;
                        break;
                    case STILL:
                        currentState = states.WORKING;
                        break;
                }
                pA = false;
            }

            if(currentState == states.WORKING) {
                Vector2d targetVec1 = new Vector2d(DISTANCE, 0);
                Vector2d targetVec2 = new Vector2d(DISTANCE, DISTANCE);
                Vector2d targetVec3 = new Vector2d(0, DISTANCE);
                Vector2d targetVec4 = new Vector2d(0, 0);
                Trajectory targetTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(targetVec1)
                        .lineTo(targetVec2)
                        .lineTo(targetVec3)
                        .lineTo(targetVec4 )
                        .build();


                drive.followTrajectory(targetTraj);
            }

            updateDashboard(drive.getPoseEstimate());
            telemetry.addData("Status: ", currentState);
            telemetry.update();
        }
    }

    private static void updateDashboard(Pose2d currentLocation) {
        TelemetryPacket telemPacket = new TelemetryPacket();
        Canvas ftcField = telemPacket.fieldOverlay();
        DashboardUtil.drawRobot(ftcField, currentLocation);
        dashboard.sendTelemetryPacket(telemPacket);
    }

}